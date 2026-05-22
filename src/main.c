// Pico SDK libraries
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
// C common libraries
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
// MAVLink libraries
#include <mavlink/common/mavlink.h>
#include <mavlink/minimal/mavlink.h>
#include <mavlink/standard/mavlink.h>
// Custom Libraries
#include "vectors.h"
#include "hardware.h"

// Global variables
// MAVLINK incoming
mavlink_status_t status;
mavlink_message_t msg;
mavlink_attitude_t attitude;
mavlink_command_ack_t ack;
mavlink_global_position_int_t position;
// MAVLINK outgoing
mavlink_message_t request_stream, pause, unpause, correct, correct_resume, pico_heartbeat, change_yaw;
const int32_t correct_alt = 50;
const uint8_t system_id = 1, component_id_mc = 200, component_id_fc = 1, chan = MAVLINK_COMM_2, UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
// UART handling parameters
volatile uint8_t byte;

// Core instruction enums
core0_comms_t core1_instruction = IDLE; 
core1_comms_t core0_instruction = RUNNING;

// Core 1 variables and flags (for safety)
volatile int32_t lattitude, longitude;
struct repeating_timer timer;
volatile bool ping = false;
// Core 0 variables and flags (for safety)
volatile bool wait = false, first_message = true, calculate = false, correcting = false; 

// Coords setup + geofence as a global variable
coords_t geofence_coords[] = {{549130910, 97803710}, {549128980, 97807300}, {549127420, 97804640}, {549129970, 97801600}};
geofence_t geofence = {sizeof(geofence_coords)/sizeof(coords_t), NULL, {INT32_MAX, INT32_MIN}, {INT32_MAX, INT32_MIN}, 0, 0};

// Function prototypes
void init(void);
bool check_geofence(mavlink_global_position_int_t);
void calculate_return(int32_t, int32_t);

// FIFO interrupt handler for the secondary computing core
void core1_FIFO(){
    while(multicore_fifo_rvalid()){
        core1_instruction = multicore_fifo_pop_blocking();
        if(core1_instruction == CALCULATE_RETURN){
            lattitude = multicore_fifo_pop_blocking();
            longitude = multicore_fifo_pop_blocking();
        }                
    }
    multicore_fifo_clear_irq();
}

bool alarm_callback(__unused struct repeating_timer *t){
    ping = true;
}

// FIFO interrupt handler for the primary messaging core
void core0_FIFO(){
    while(multicore_fifo_rvalid()){
        core0_instruction = multicore_fifo_pop_blocking();
    }
    multicore_fifo_clear_irq();
}


// UART recieve interrupt
void on_uart_rx(){
    while(uart_is_readable(UART_ID)){
            byte = uart_getc(UART_ID);

        if(mavlink_parse_char(chan, byte, &msg, &status)){
            if(first_message){
                multicore_fifo_push_blocking(SEND_STREAM_REQ);
                first_message = false;
            }
            // printf("ID %d\n", msg.msgid);
            switch(msg.msgid){                
                case(MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
                    mavlink_msg_global_position_int_decode(&msg, &position);
                    watchdog_update();

                    if(!(check_geofence(position)) && !correcting){
                        multicore_fifo_push_blocking(SEND_PAUSE);
                        calculate = true;
                        correcting = true;
                        printf("\nOUTSIDE\n"); 
                    }
                    printf("\nPosition recieved, %d, %d, %u\n", position.lat, position.lon, position.hdg);
                    break;

                case(MAVLINK_MSG_ID_COMMAND_ACK):
                    mavlink_msg_command_ack_decode(&msg, &ack);
                    watchdog_update();
                    switch(ack.command){
                        case(MAV_CMD_DO_PAUSE_CONTINUE):
                            if(ack.result == MAV_RESULT_ACCEPTED){
                                printf("\nPause recieved\n");
                            }
                            break;

                        case(MAV_CMD_CONDITION_YAW):
                            if(ack.result == MAV_RESULT_ACCEPTED){
                                    printf("\nYaw correction acknowledged\n");
                                    multicore_fifo_push_blocking(SEND_UNPAUSE);
                                    wait = true;
                                }
                                break;
                            
                        case(MAV_CMD_OVERRIDE_GOTO):
                            if(ack.result == MAV_RESULT_ACCEPTED && wait){
                                multicore_fifo_push_blocking(SEND_RESUME);
                                wait = false;
                                printf("\nGOTO recieved\n");
                            }
                            else if(ack.result == MAV_RESULT_ACCEPTED && !wait){
                                correcting = false;
                            }
                            break;
                    }
                }
                break;
            }
        }
    watchdog_update();
    }

void main(){
    init();

    while(true){
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        watchdog_update();
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
        watchdog_update();
        if(calculate){

            sleep_ms(500);
            watchdog_update();
            sleep_ms(500);
            watchdog_update();

            multicore_fifo_push_blocking(CALCULATE_RETURN);
            multicore_fifo_push_blocking(position.lat);
            multicore_fifo_push_blocking(position.lon);
            calculate = false;
        }
    }   

    free(geofence.waypoints);
}

void core1_entry(){
    usart_init();

    // Sets up which function to use for the FIFO interrupt between core 0 to 1
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_FIFO);
    irq_set_enabled(SIO_IRQ_PROC1, true);

    // Adds a callback timer to accomodate a heartbeat signal
    add_repeating_timer_ms(1000, alarm_callback, NULL, &timer);

    while(true){
        switch(core1_instruction){
            case(SEND_STREAM_REQ):
            core1_instruction = IDLE;
            send_mav(&request_stream);
            break;

            case(SEND_PAUSE):
            core1_instruction = IDLE;
            send_mav(&pause);
            break;

            case(CALCULATE_RETURN):
            core1_instruction = IDLE;
            calculate_return(lattitude, longitude);
            break;

            case(SEND_RESUME):
            core1_instruction = IDLE;
            send_mav(&correct_resume);
            break;

            case(SEND_UNPAUSE):
            core1_instruction = IDLE;
            send_mav(&unpause);
            send_mav(&correct);
            break;
        }
        if(ping){send_mav(&pico_heartbeat);}
    }
}

void init(void){
    usart_init();

    // Clears the fifo register and enables the core 0 interrupt
    multicore_fifo_clear_irq();
    irq_set_enabled(SIO_IRQ_PROC0, true);

    // Defines the core0 fifo interrupt handler
    irq_set_exclusive_handler(SIO_IRQ_PROC0, core0_FIFO);

    // Launches the second core with its main function
    multicore_launch_core1(core1_entry); 

    // And set up and enable the UART interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Defines the stop messages and the request datastream message
    mavlink_msg_command_long_pack(system_id, component_id_mc, &request_stream, system_id, component_id_fc, MAV_CMD_SET_MESSAGE_INTERVAL, (uint8_t)(0), (float)(MAVLINK_MSG_ID_GLOBAL_POSITION_INT), (float)(100000), (float)(0), (float)(0), (float)(0), (float)(0), (float)(1));
    mavlink_msg_heartbeat_pack(system_id, component_id_mc, &pico_heartbeat, MAV_TYPE_ONBOARD_CONTROLLER, (uint8_t)MAV_AUTOPILOT_INVALID, (uint8_t)MAV_MODE_FLAG_AUTO_ENABLED, (uint32_t)0, (uint8_t)MAV_STATE_ACTIVE);
    mavlink_msg_command_long_pack(system_id, component_id_mc, &pause, system_id, component_id_fc, MAV_CMD_DO_PAUSE_CONTINUE, (uint8_t)(0), (float)(MAV_BOOL_FALSE), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0)); // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE 
    mavlink_msg_command_long_pack(system_id, component_id_mc, &unpause, system_id, component_id_fc, MAV_CMD_DO_PAUSE_CONTINUE, (uint8_t)(0), (float)(MAV_BOOL_TRUE), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0)); 

    geofence.waypoints = malloc(geofence.num_of_waypoints * sizeof(coords_t)); // Allocates pointer memory to the waypoints data in the geofence struct

for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){ // Loads up the waypoints into the struct
        geofence.waypoints[i] = geofence_coords[i];
        geofence.latt_avg += ((double)geofence.waypoints[i].latt_e7 / (double)geofence.num_of_waypoints);
        geofence.long_avg += ((double)geofence.waypoints[i].long_e7 / (double)geofence.num_of_waypoints);
    }

    // printf("Fence Center: %.7f %.7f\n", geofence.latt_avg / 1e7, geofence.long_avg / 1e7);

    for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){ // Corrects the soft geofence for offset which is defined at the beginning
        coords_t coords = {geofence.waypoints[i].latt_e7, geofence.waypoints[i].long_e7}, centre = {geofence.latt_avg, geofence.long_avg};
        vect3D_t P, PC, C;

        P = spherical_to_euclid(coords);
        C = spherical_to_euclid(centre);
        PC = vect_subtract(C, P);
        vect_normalize(&PC);
        vect_multiply(&PC, INIT_GEOF_OFFSET_M);
        P = vect_add(P, PC);
        geofence.waypoints[i] = euclid_to_spherical(P);

        // printf("\n%d, %d, geofence offset waypoint %d\n", geofence.waypoints[i].latt_e7, geofence.waypoints[i].long_e7, i);

        if(geofence.extremas_long[0] > geofence.waypoints[i].long_e7){geofence.extremas_long[0] = geofence.waypoints[i].long_e7;}
        if(geofence.extremas_long[1] < geofence.waypoints[i].long_e7){geofence.extremas_long[1] = geofence.waypoints[i].long_e7;}
        if(geofence.extremas_latt[0] > geofence.waypoints[i].latt_e7){geofence.extremas_latt[0] = geofence.waypoints[i].latt_e7;}
        if(geofence.extremas_latt[1] < geofence.waypoints[i].latt_e7){geofence.extremas_latt[1] = geofence.waypoints[i].latt_e7;}
    }
        watchdog_enable(1000, 1);
        watchdog_update();
}

// Function which checks the coordinate and whether it lies in the defined geofence
bool check_geofence(mavlink_global_position_int_t pos){
    watchdog_update();
    bool inside = false;

    // Bounding box check
    // Checks whether the point doesnt lie outside the extremas of the fence making it lie outside by default
    if(pos.lat < geofence.extremas_latt[0] || pos.lat >  geofence.extremas_latt[1] || pos.lon <  geofence.extremas_long[0] || pos.lon > geofence.extremas_long[1]){
        return inside;
    }

    // Line intercept algorithm
    // Checks whether the coordinates lie on the longitude range of a given pair of points from the geofence and checks whether a line drawn in the positive lattitude direction intersects
    // Based on whether 1 or 2 / 0 intersections are found we can know whether we are inside or ourside the fence
    for(uint8_t i = 0, j = geofence.num_of_waypoints - 1; i < geofence.num_of_waypoints; j = i++){
        if(((pos.lon < geofence.waypoints[i].long_e7) != (pos.lon < geofence.waypoints[j].long_e7)) && (pos.lat < (int32_t)round(((double)(geofence.waypoints[i].latt_e7 - geofence.waypoints[j].latt_e7) / (double)(geofence.waypoints[i].long_e7 - geofence.waypoints[j].long_e7)) * (double)(pos.lon - geofence.waypoints[i].long_e7) + (double)geofence.waypoints[i].latt_e7))){
            inside = !inside;
        }
        watchdog_update();
    }
    return inside;
}


// Proposed function, to let the drone wait after pause, and calculate some manner of valid return coordinate into the geofence before mission continuation
void calculate_return(int32_t lat, int32_t lon){
    double min_dist = UINT64_MAX, dist, proj_const;
    coords_t point = {lat, lon}, centre = {geofence.latt_avg, geofence.long_avg}, return_coords, return_coords_offset, correct_coords, correct_offset;
    vect3D_t A, B, P, AB, AP, C, EC, E;

    for(uint8_t i = 0, j = geofence.num_of_waypoints - 1; i < geofence.num_of_waypoints; j = i++){
        A = spherical_to_euclid(geofence.waypoints[j]);
        B = spherical_to_euclid(geofence.waypoints[i]);
        P = spherical_to_euclid(point);
        C = spherical_to_euclid(centre);
        AB = vect_subtract(B, A);
        AP = vect_subtract(P, A);
        proj_const = project(AP, AB);

        if(proj_const < 0){proj_const = 0;}
        if(proj_const > 1){proj_const = 1;}

        vect_multiply(&AB, proj_const);
        E = vect_add(AB, A);
        EC = vect_subtract(C, E);
        vect_normalize(&EC);
        vect_multiply(&EC, GEOF_OFFSET_M);
        return_coords = euclid_to_spherical(E);
        E = vect_add(E, EC);
        return_coords_offset = euclid_to_spherical(E);

        dist = distance(return_coords, point);

        if(dist < min_dist){
            correct_offset = return_coords_offset; 
            correct_coords = return_coords;
            min_dist = dist;
        }
    }

    vect3D_t Up = P, Z = {0, 0, 1};
    vect_normalize(&Up);
    vect3D_t East = cross_prod(Up, Z);
    vect_normalize(&East);
    vect3D_t North = cross_prod(East, Up);
    vect_normalize(&North);
    vect3D_t PC = vect_subtract(C, P), East_comp = East, North_comp = North;
    float correct_yaw = (float)atan2(dot(PC, East), dot(PC, North));

    // This information is packed into a MAVLink command interrupt GOTO and an unpause message is sent
    mavlink_msg_command_long_pack(system_id, component_id_mc, &correct, system_id, component_id_fc, MAV_CMD_OVERRIDE_GOTO, (uint8_t)(0), (float)(MAV_GOTO_DO_HOLD), (float)(MAV_GOTO_HOLD_AT_SPECIFIED_POSITION), (float)(MAV_FRAME_GLOBAL_INT), (float)(0), (float)(correct_offset.latt_e7), (float)(correct_offset.long_e7), (float)(correct_alt)); 
    mavlink_msg_command_long_pack(system_id, component_id_mc, &change_yaw, system_id, component_id_fc, MAV_CMD_CONDITION_YAW, (uint8_t)(0), (float)(correct_yaw), (float)(180), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0));
    send_mav(&change_yaw);
}