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
mavlink_mission_count_t mission_count;
mavlink_mission_item_int_t mission_item;
// MAVLINK outgoing
mavlink_message_t request_stream, pause, unpause, correct, correct_resume, pico_heartbeat, change_yaw, mission_list_req;
const int32_t correct_alt = 50;
const uint8_t system_id = 1, component_id_mc = 200, component_id_fc = 1, chan = MAVLINK_COMM_2, UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
// UART handling parameters
volatile uint8_t byte;

// Core instruction enums
volatile core0_comms_t core1_instruction = IDLE; 
volatile core1_comms_t core0_instruction = RUNNING;

// Core 1 variables and flags (for safety)
volatile int32_t lattitude, longitude;
volatile uint16_t num_of_waypoints;
struct repeating_timer timer;
volatile bool ping = false, change = false;
mavlink_message_t sys_status;
float param1_flt = 0, param2_flt = 0, param3_flt = 0, param4_flt = 0;
uint16_t param1_uint = 0;
const uint8_t system_id_debug = 20;
// Core 0 variables and flags (for safety)
volatile bool wait = false, first_message = true, calculate = false, correcting = false; 
volatile int16_t param1_int = 1;

// Coords setup + geofence as a global variable
coords_t * mission_coords;
coords_t geofence_coords[] = {{549228230, 98164190}, {549230240, 98163540}, {549230600, 98168310}, {549228600, 98167910}};
geofence_t geofence = {sizeof(geofence_coords)/sizeof(coords_t), NULL, {INT32_MAX, INT32_MIN}, {INT32_MAX, INT32_MIN}, 0, 0};

// Function prototypes
void init(void);
bool check_geofence(mavlink_global_position_int_t);
void calculate_return(int32_t, int32_t);
void request_mission(uint16_t);
void geofence_setup(void);
void update_system(void); 

// FIFO interrupt handler for the secondary computing core
void core1_FIFO(){
    while(multicore_fifo_rvalid()){
        core1_instruction = multicore_fifo_pop_blocking();
        if(core1_instruction == CALCULATE_RETURN){
            lattitude = multicore_fifo_pop_blocking();
            longitude = multicore_fifo_pop_blocking();
        }
        else if(core1_instruction == REQUEST_FENCE){
            num_of_waypoints = multicore_fifo_pop_blocking();
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
            printf("ID %d\n", msg.msgid);
            switch(msg.msgid){                
                case(MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
                    mavlink_msg_global_position_int_decode(&msg, &position);
                    watchdog_update();

                    if(!(check_geofence(position)) && !correcting){
                        multicore_fifo_push_blocking(SEND_PAUSE);
                        calculate = true;
                        correcting = true;
                        printf("\nOUTSIDE\n"); 
                        param1_int = 0;
                    }
                    printf("\nPosition recieved, %d, %d, %u\n", position.lat, position.lon, position.hdg);
                    break;

                case(MAVLINK_MSG_ID_MISSION_COUNT):
                    mavlink_msg_mission_count_decode(&msg, &mission_count);
                    watchdog_update();
                    multicore_fifo_push_blocking(REQUEST_FENCE);
                    multicore_fifo_push_blocking(mission_count.count);
                break;

                case(MAVLINK_MSG_ID_ATTITUDE):
                    mavlink_msg_attitude_decode(&msg, &attitude); 
                    watchdog_update();
                break;

                case(MAVLINK_MSG_ID_MISSION_ITEM_INT):
                    mavlink_msg_mission_item_int_decode(&msg, &mission_item);
                    printf("%d\n", mission_item.frame);
                    if(mission_item.frame == MAV_MISSION_TYPE_FENCE){
                        mission_coords[mission_item.seq] = (coords_t){mission_item.x, mission_item.y};
                        if(mission_item.seq == mission_count.count){
                            // multicore_fifo_push_blocking(SETUP_GEOFENCE);
                        }
                    }
                    watchdog_update();
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
                                param1_int = 1;
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
        printf("blinky\n");
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
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_FIFO);
    irq_set_enabled(SIO_IRQ_PROC1, true);
    stdio_init_all();
    pico_led_init();

    // Sets up UART registers
    uart_init(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Set the TX and RX pins by using the function select on the GPIO
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    add_repeating_timer_ms(1000, alarm_callback, NULL, &timer);

    while(true){
        switch(core1_instruction){
            case(SEND_STREAM_REQ):
            core1_instruction = IDLE;
            send_mav(&request_stream);
            send_mav(&mission_list_req);
            break;

            case(SEND_PAUSE):
            param1_uint = PAUSE_SENDING;
            core1_instruction = IDLE;
            send_mav(&pause);
            break;

            case(CALCULATE_RETURN):
            param1_uint = RETURN_CALCULATING;
            core1_instruction = IDLE;
            calculate_return(lattitude, longitude);
            break;

            case(SEND_RESUME):
            param1_uint = SENDING_RESUME;
            core1_instruction = IDLE;
            send_mav(&correct_resume);
            break;

            case(SEND_UNPAUSE):
            param1_uint = SENDING_UNPAUSE;
            core1_instruction = IDLE;
            send_mav(&unpause);
            send_mav(&correct);
            break;

            case(REQUEST_FENCE):
            param1_uint = REQUESTING_MISSION;
            core1_instruction = IDLE;
            request_mission(num_of_waypoints);
            break;

            case(SETUP_GEOFENCE):
            param1_uint = SETTING_UP_GEOFENCE;
            core1_instruction = IDLE;
            geofence_setup();
            break;
        }
        if(change){
            update_system();
        }
        if(ping){
            send_mav(&pico_heartbeat);
            send_mav(&sys_status);
        }
    }
}

void init(void){
    stdio_init_all();
    pico_led_init();

    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC0, core0_FIFO);
    irq_set_enabled(SIO_IRQ_PROC0, true);

    multicore_launch_core1(core1_entry); // Launches the second core with its main function

    // Sets up UART registers
    uart_init(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, true);

    // Set the TX and RX pins by using the function select on the GPIO
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // Defines the stop messages and the request datastream message
    mavlink_msg_command_long_pack(system_id, component_id_mc, &request_stream, system_id, component_id_fc, MAV_CMD_SET_MESSAGE_INTERVAL, (uint8_t)(0), (float)(MAVLINK_MSG_ID_GLOBAL_POSITION_INT), (float)(100000), (float)(0), (float)(0), (float)(0), (float)(0), (float)(1));
    mavlink_msg_heartbeat_pack(system_id, component_id_mc, &pico_heartbeat, MAV_TYPE_ONBOARD_CONTROLLER, (uint8_t)MAV_AUTOPILOT_INVALID, (uint8_t)MAV_MODE_FLAG_AUTO_ENABLED, (uint32_t)0, (uint8_t)MAV_STATE_ACTIVE);
    mavlink_msg_command_long_pack(system_id, component_id_mc, &pause, system_id, component_id_fc, MAV_CMD_DO_PAUSE_CONTINUE, (uint8_t)(0), (float)(MAV_BOOL_FALSE), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0)); // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE 
    mavlink_msg_command_long_pack(system_id, component_id_mc, &unpause, system_id, component_id_fc, MAV_CMD_DO_PAUSE_CONTINUE, (uint8_t)(0), (float)(MAV_BOOL_TRUE), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0)); 
    mavlink_msg_mission_request_list_pack(system_id, component_id_mc, &mission_list_req, system_id, component_id_fc, MAV_MISSION_TYPE_FENCE);

    geofence.waypoints = malloc(geofence.num_of_waypoints * sizeof(coords_t)); // Allocates pointer memory to the waypoints data in the geofence struct

    for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){ // Loads up the waypoints into the struct
        geofence.waypoints[i] = geofence_coords[i];
        geofence.latt_avg += ((double)geofence.waypoints[i].latt_e7 / (double)geofence.num_of_waypoints);
        geofence.long_avg += ((double)geofence.waypoints[i].long_e7 / (double)geofence.num_of_waypoints);
    }

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

        if(geofence.extremas_long[0] > geofence.waypoints[i].long_e7){geofence.extremas_long[0] = geofence.waypoints[i].long_e7;}
        if(geofence.extremas_long[1] < geofence.waypoints[i].long_e7){geofence.extremas_long[1] = geofence.waypoints[i].long_e7;}
        if(geofence.extremas_latt[0] > geofence.waypoints[i].latt_e7){geofence.extremas_latt[0] = geofence.waypoints[i].latt_e7;}
        if(geofence.extremas_latt[1] < geofence.waypoints[i].latt_e7){geofence.extremas_latt[1] = geofence.waypoints[i].latt_e7;}
    }

    // watchdog_enable(1000, 1);
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
    vect3D_t A, B, P, AB, AP, C, EC, E, E_corr;

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
            E_corr = E;
            correct_offset = return_coords_offset; 
            correct_coords = return_coords;
            min_dist = dist;
        }
    }

    vect3D_t Up = P, Z = {0, 0, 1};
    vect_normalize(&Up);
    vect3D_t East = cross_prod(Z, Up);
    vect_normalize(&East);
    vect3D_t North = cross_prod(Up, East);
    vect_normalize(&North);
    vect3D_t PE = vect_subtract(E_corr, P);
    float correct_yaw = RAD2DEG((float)atan2(dot(PE, East), dot(PE, North)));

    param1_flt = correct_yaw;
    param3_flt = correct_offset.latt_e7 / 1e7;
    param4_flt = correct_offset.long_e7 / 1e7;
    change = true;

    // This information is packed into a MAVLink command interrupt GOTO and an unpause message is sent
    mavlink_msg_command_long_pack(system_id, component_id_mc, &correct, system_id, component_id_fc, MAV_CMD_OVERRIDE_GOTO, (uint8_t)(0), (float)(MAV_GOTO_DO_HOLD), (float)(MAV_GOTO_HOLD_AT_SPECIFIED_POSITION), (float)(MAV_FRAME_GLOBAL_INT), (float)(0), (float)(correct_offset.latt_e7), (float)(correct_offset.long_e7), (float)(correct_alt)); 
    mavlink_msg_command_long_pack(system_id, component_id_mc, &change_yaw, system_id, component_id_fc, MAV_CMD_CONDITION_YAW, (uint8_t)(0), (float)(correct_yaw), (float)(180), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0));
    send_mav(&change_yaw);
}

void request_mission(uint16_t num){
    mavlink_mission_request_int_t mission_item_request;
    mission_coords = malloc(num * sizeof(coords_t));

    // for(uint16_t i = 0; i < num; i++){
    //     mavlink_msg_mission_request_int_pack(system_id, component_id_mc, &mission_item_request, system_id, component_id_fc, i, MAV_MISSION_TYPE_FENCE);
    //     send_mav(&mission_item_request);
    // }
}

void geofence_setup(void){
    geofence.waypoints = malloc(num_of_waypoints * sizeof(coords_t)); // Allocates pointer memory to the waypoints data in the geofence struct

    for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){ // Loads up the waypoints into the struct
            geofence.waypoints[i] = mission_coords[i];
            geofence.latt_avg += ((double)geofence.waypoints[i].latt_e7 / (double)geofence.num_of_waypoints);
            geofence.long_avg += ((double)geofence.waypoints[i].long_e7 / (double)geofence.num_of_waypoints);
        }

        free(mission_coords);

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

            if(geofence.extremas_long[0] > geofence.waypoints[i].long_e7){geofence.extremas_long[0] = geofence.waypoints[i].long_e7;}
            if(geofence.extremas_long[1] < geofence.waypoints[i].long_e7){geofence.extremas_long[1] = geofence.waypoints[i].long_e7;}
            if(geofence.extremas_latt[0] > geofence.waypoints[i].latt_e7){geofence.extremas_latt[0] = geofence.waypoints[i].latt_e7;}
            if(geofence.extremas_latt[1] < geofence.waypoints[i].latt_e7){geofence.extremas_latt[1] = geofence.waypoints[i].latt_e7;}
        }

    }

    void update_system(void){
        mavlink_msg_vfr_hud_pack(system_id_debug, component_id_mc, &sys_status, (float)param1_flt, (float)param2_flt, (int16_t)param1_int, (uint16_t)param1_uint, (float)param3_flt, (float)param4_flt);
        change = false;
    }