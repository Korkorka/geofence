// Pico SDK libraries
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
// C common libraries
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
// MAVLink libraries
#include <mavlink/common/mavlink.h>
#include <mavlink/minimal/mavlink.h>
#include <mavlink/standard/mavlink.h>
// UART and MAVLink setup + LED blink
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define LED_DELAY_MS 500

// Definition for safe Geofence offset and calculation for the degree offset, approximates the offset as in cartesians
#define geof_offset_m 1
#define geof_prec 3
const double earth_approx_rad = 6371000;
const int32_t geof_offset_deg = (int32_t)(((double)(geof_offset_m * 180) / (earth_approx_rad * M_PI)) * 1e7);

// enum PX4_CUSTOM_MAIN_MODE {
// 	PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
// 	PX4_CUSTOM_MAIN_MODE_ALTCTL,
// 	PX4_CUSTOM_MAIN_MODE_POSCTL,
// 	PX4_CUSTOM_MAIN_MODE_AUTO,
// 	PX4_CUSTOM_MAIN_MODE_ACRO,
// 	PX4_CUSTOM_MAIN_MODE_OFFBOARD,
// 	PX4_CUSTOM_MAIN_MODE_STABILIZED,
// 	PX4_CUSTOM_MAIN_MODE_RATTITUDE_LEGACY,
// 	PX4_CUSTOM_MAIN_MODE_SIMPLE, /* unused, but reserved for future use */
// 	PX4_CUSTOM_MAIN_MODE_TERMINATION,
// 	PX4_CUSTOM_MAIN_MODE_ALTITUDE_CRUISE
// };

// enum PX4_CUSTOM_SUB_MODE_AUTO {
// 	PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
// 	PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
// 	PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
// 	PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
// 	PX4_CUSTOM_SUB_MODE_AUTO_RTL,
// 	PX4_CUSTOM_SUB_MODE_AUTO_LAND,
// 	PX4_CUSTOM_SUB_MODE_AUTO_RESERVED_DO_NOT_USE, // was PX4_CUSTOM_SUB_MODE_AUTO_RTGS, deleted 2020-03-05
// 	PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,
// 	PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND,
// 	PX4_CUSTOM_SUB_MODE_AUTO_VTOL_TAKEOFF,
// 	PX4_CUSTOM_SUB_MODE_EXTERNAL1,
// 	PX4_CUSTOM_SUB_MODE_EXTERNAL2,
// 	PX4_CUSTOM_SUB_MODE_EXTERNAL3,
// 	PX4_CUSTOM_SUB_MODE_EXTERNAL4,
// 	PX4_CUSTOM_SUB_MODE_EXTERNAL5,
// 	PX4_CUSTOM_SUB_MODE_EXTERNAL6,
// 	PX4_CUSTOM_SUB_MODE_EXTERNAL7,
// 	PX4_CUSTOM_SUB_MODE_EXTERNAL8,
// };

typedef struct{ // Coordinate struct with latt and long expressed multplied by 1e7
    int32_t delta_lat_e7;
    int32_t phi_long_e7;
}coords_t; 

typedef struct{ // Geofence struct with adaptive sizing
    coords_t* waypoints;
    int32_t extremas_long[2];
    int32_t extremas_latt[2];
    double long_avg; 
    double latt_avg;
    uint8_t num_of_waypoints;
}geofence_t;

// MAVLINK recieve definitions
mavlink_status_t status;
mavlink_message_t msg;
mavlink_command_ack_t ack;
mavlink_heartbeat_t heartbeat;
mavlink_global_position_int_t position;
// MAVLINK commands
mavlink_message_t request_stream, hover_mode, mission_mode, pause, unpause, correct, correct_resume, pico_heartbeat;
const int32_t correct_alt = 50;
// const enum PX4_CUSTOM_MAIN_MODE automatic = PX4_CUSTOM_MAIN_MODE_AUTO;
// const enum PX4_CUSTOM_SUB_MODE_AUTO hover = PX4_CUSTOM_SUB_MODE_AUTO_LOITER, mission = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
const uint8_t system_id = 1, component_id_mc = 200, component_id_fc = 1, chan = MAVLINK_COMM_2, UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
// UART handling params
volatile uint8_t byte;
volatile bool wait = false, first_message = true;   

// Coords setup + geofence as a global variable
coords_t coords[] = {{549130910, 97803710}, {549128980, 97807300}, {549127420, 97804640}, {549129970, 97801600}};
geofence_t geofence = {sizeof(coords)/sizeof(coords_t), NULL, {0, 0}, {0, 0}, 0, 0};

// Function prototypes
int pico_led_init(void);
void pico_set_led(bool);
void init(void);
void send_mav(mavlink_message_t*);
bool check_geofence(mavlink_global_position_int_t);
void calculate_return_coords(mavlink_global_position_int_t);

// UART recieve interrupt
void on_uart_rx(){
    while(uart_is_readable(UART_ID)){
            byte = uart_getc(UART_ID);

        if(mavlink_parse_char(chan, byte, &msg, &status)){
            printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);

            switch(msg.msgid){
                case(MAVLINK_MSG_ID_HEARTBEAT): 
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                // watchdog_update();
                //printf("\nHearbeat\n");
                if(first_message){send_mav(&request_stream);}
                break;
                
                case(MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
                mavlink_msg_global_position_int_decode(&msg, &position);
                // watchdog_update();
                //printf("\nPosition recieved\n");
                if(!(check_geofence(position))){
                    //printf("\nOUTSIDE\n"); // For geofence manual testing
                    send_mav(&pause);
                    wait = true;
                    calculate_return_coords(position);
                }
                break;

                case(MAVLINK_MSG_ID_COMMAND_ACK):
                mavlink_msg_command_ack_decode(&msg, &ack);
                // watchdog_update();
                switch(ack.command){
                    case(MAV_CMD_DO_PAUSE_CONTINUE):
                    //printf("\nPause recieved\n");
                    if(ack.result == MAV_RESULT_ACCEPTED || ack.result == MAV_RESULT_IN_PROGRESS){
                        if(wait){
                            wait = false;
                        }
                        else if(!wait){
                            send_mav(&correct);
                            wait = true;
                        }
                    }
                    break;
                    case(MAV_CMD_OVERRIDE_GOTO):
                    //printf("\nGOTO recieved\n");
                    if(ack.result == MAV_RESULT_ACCEPTED && wait){
                        send_mav(&correct_resume);
                        wait = false;
                        break;
                    }
                    else if(ack.result == MAV_RESULT_IN_PROGRESS || ack.result == MAV_RESULT_ACCEPTED){
                        break;
                    }
                    else{
                        send_mav(&correct);
                        break;
                    }
                }
                break;
            }
        }
    // watchdog_update();
    // printf("%d", status.parse_state); // from an earlier debug session (state of parsing 1 = idle 1-15 = parsing)
    }
}

int main(){
    init();

    if (watchdog_enable_caused_reboot()) {
        printf("Watchdog oopsie\n");
        return 0;
    }

    while(true){
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        // watchdog_update();
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
        // watchdog_update();
        send_mav(&pico_heartbeat);
    }   

    free(geofence.waypoints);
}

void init(void){
    stdio_init_all();
    pico_led_init();

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
    
    //Enables the watchdog timer at 1s delay
    // watchdog_enable(1000, 1);

    // Defines the stop messages and the request datastream message
    mavlink_msg_request_data_stream_pack(system_id, component_id_mc, &request_stream, system_id, component_id_fc, MAV_DATA_STREAM_ALL, (uint16_t)10, ((uint8_t)1));
    mavlink_msg_heartbeat_pack((uint8_t)system_id, (uint8_t)component_id_mc, &pico_heartbeat, (uint8_t)MAV_TYPE_ONBOARD_CONTROLLER, (uint8_t)MAV_AUTOPILOT_INVALID, (uint8_t)MAV_MODE_FLAG_AUTO_ENABLED, (uint32_t)0, (uint8_t)MAV_STATE_ACTIVE);
    // mavlink_msg_command_long_pack(system_id, component_id_mc, &hover_mode, system_id, component_id_fc, MAV_CMD_DO_SET_MODE, (uint8_t)(0), (float)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED), (float)(automatic), (float)(hover), (float)(0), (float)(0), (float)(0), (float)(0)); // https://mavlink.io/en/messages/common.html#mav_commands, https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE
    // mavlink_msg_command_long_pack(system_id, component_id_mc, &mission_mode, system_id, component_id_fc, MAV_CMD_DO_SET_STANDARD_MODE, (uint8_t)(0), (float)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED), (float)(automatic), (float)(mission), (float)(0), (float)(0), (float)(0), (float)(0)); 
    mavlink_msg_command_long_pack(system_id, component_id_mc, &pause, system_id, component_id_fc, MAV_CMD_DO_PAUSE_CONTINUE, (uint8_t)(0), (float)(MAV_BOOL_FALSE), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0)); // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE 
    mavlink_msg_command_long_pack(system_id, component_id_mc, &unpause, system_id, component_id_fc, MAV_CMD_DO_PAUSE_CONTINUE, (uint8_t)(0), (float)(MAV_BOOL_TRUE), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0)); 

    geofence.waypoints = malloc(geofence.num_of_waypoints * sizeof(coords_t)); // Allocates pointer memory to the waypoints data in the geofence struct

    for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){ // Loads up the waypoints into the struct
        geofence.waypoints[i] = coords[i];
        geofence.latt_avg += geofence.waypoints[i].delta_lat_e7 / geofence.num_of_waypoints;
        geofence.long_avg += geofence.waypoints[i].phi_long_e7 / geofence.num_of_waypoints;
        // watchdog_update();
    }


    for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){ // Corrects the soft geofence for offset which is defined at the beginning
        geofence.waypoints[i]; 
        if(geofence.waypoints[i].delta_lat_e7 < geofence.latt_avg){
            geofence.waypoints[i].delta_lat_e7 += geof_offset_deg;
        }
        else{
            geofence.waypoints[i].delta_lat_e7 -= geof_offset_deg;
        }
        if(geofence.waypoints[i].phi_long_e7 < geofence.long_avg){
            geofence.waypoints[i].phi_long_e7 += geof_offset_deg;
        }
        else{
            geofence.waypoints[i].phi_long_e7 -= geof_offset_deg;
        }
        if(geofence.extremas_long[0] > geofence.waypoints[i].phi_long_e7){geofence.extremas_long[0] = geofence.waypoints[i].phi_long_e7;}
        if(geofence.extremas_long[1] < geofence.waypoints[i].phi_long_e7){geofence.extremas_long[1] = geofence.waypoints[i].phi_long_e7;}
        if(geofence.extremas_latt[0] > geofence.waypoints[i].delta_lat_e7){geofence.extremas_latt[0] = geofence.waypoints[i].delta_lat_e7;}
        if(geofence.extremas_latt[1] < geofence.waypoints[i].delta_lat_e7){geofence.extremas_latt[1] = geofence.waypoints[i].delta_lat_e7;}
    }
}

// Perform LED initialisation
int pico_led_init(void){
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on){
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

// Quick message for putting a MAVLink message to UART in an array of bytes
void send_mav(mavlink_message_t* msg){
    char tx_arr[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_to_send_buffer(tx_arr, msg);
    uart_write_blocking(UART_ID, tx_arr, MAVLINK_MAX_PACKET_LEN);
}

// Function which checks the coordinate and whether it lies in the defined geofence
bool check_geofence(mavlink_global_position_int_t pos){
    int32_t pla = pos.lat, plo = pos.lon;

    // Bounding box check
    // Checks whether the point doesnt lie outside the extremas of the fence making it lie outside by default
    if(pla < geofence.extremas_latt[0] || pla >  geofence.extremas_latt[1] || plo <  geofence.extremas_long[0] || plo > geofence.extremas_latt[1]){
        return true;
    }

    // Line intercept algorithm
    // Checks whether the coordinates lie on the longitude range of a given pair of points from the geofence and checks whether a line drawn in the positive lattitude direction intersects
    // Based on whether 1 or 2 / 0 intersections are found we can know whether we are inside or ourside the fence
    uint8_t intercepts = 0;
    for(uint8_t i = 0, j = geofence.num_of_waypoints - 1; i < geofence.num_of_waypoints; j = i++){
        int32_t point1_latt = geofence.waypoints[i].delta_lat_e7, point1_long = geofence.waypoints[i].phi_long_e7;
        int32_t point2_latt = geofence.waypoints[j].delta_lat_e7, point2_long = geofence.waypoints[j].phi_long_e7;

        if(((plo < point1_long) != (plo < point2_long)) && (pla < (int32_t)round(((point1_latt - point2_latt) / (point1_long - point2_long)) * (plo - point1_latt) + point1_latt))){intercepts++;}
        // watchdog_update();
    }
    bool check = true;
    if(intercepts % 2){check = false;}

    return check;
}


// Proposed function, to let the drone wait after pause, and calculate some manner of valid return coordinate into the geofence before mission continuation
void calculate_return_coords(mavlink_global_position_int_t pos){
    uint64_t min_square_dist = UINT64_MAX, sec_min_squared_dist = UINT64_MAX;
    uint64_t square_dist;
    int32_t avg_latt[geof_prec + 2], avg_long[geof_prec + 2], correct_latt, correct_long;

    for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){ // This loop finds the two closest geofence waypoints by compaing squared distance in the spherical coordinate system
        int32_t point_latt = geofence.waypoints[i].delta_lat_e7, point_long = geofence.waypoints[i].phi_long_e7;
        square_dist = sqrt(pow((point_latt - pos.lat), 2) + pow((point_long - pos.lon), 2));

        if(square_dist < min_square_dist){
            avg_latt[0] = point_latt;
            avg_long[0] = point_long;
            min_square_dist = square_dist;
        }
        else if(square_dist < sec_min_squared_dist){
            avg_latt[geof_prec + 1] = point_latt;
            avg_long[geof_prec + 1] = point_long;
            sec_min_squared_dist = square_dist;
        }
        // watchdog_update();
    }

    for(uint32_t i = 2, j = (2 ^ geof_prec), f = 1; f <= (geof_prec); f++, j /= 2){ // This loop creates midpoints on a curve between the two waypoints via weighted averages and compares whether one of them isnt closer, precision defined by a parameter
        avg_latt[f] = ((double)avg_latt[0] * j + (double)avg_latt[geof_prec + 1] * i) / (i + j);
        avg_long[f] = ((double)avg_long[0] * j + (double)avg_long[geof_prec + 1] * i) / (i + j);
        square_dist = sqrt(pow((avg_latt[f] - pos.lat), 2) + pow((avg_long[f] - pos.lon), 2));

        if(square_dist < min_square_dist){
            avg_latt[0] = avg_latt[f];
            avg_long[0] = avg_long[f];
            min_square_dist = square_dist;        
        }
        // watchdog_update();
    }

    // Simple application of the original geofence offset to the coordinates, so the drone doesnt end up on the edge of the geofence
    if(geofence.latt_avg < avg_latt[0]){correct_latt = avg_latt[0] - geof_offset_deg;}
    else{correct_latt = avg_latt[0] + geof_offset_deg;}
    if(geofence.long_avg < avg_long[0]){correct_long = avg_long[0] - geof_offset_deg;}
    else{correct_long = avg_long[0] + geof_offset_deg;}
    printf("\n%d, %d, Return coords\n", correct_latt, correct_long);
    // This information is packed into a MAVLink command interrupt GOTO and an unpause message is sent
    mavlink_msg_command_long_pack(system_id, component_id_mc, &correct, system_id, component_id_fc, MAV_CMD_OVERRIDE_GOTO, (uint8_t)(0), (float)(MAV_GOTO_DO_HOLD), (float)(MAV_GOTO_HOLD_AT_SPECIFIED_POSITION), (float)(MAV_FRAME_GLOBAL_INT), (float)(0), (float)(correct_latt), (float)(correct_long), (float)(correct_alt)); 
    send_mav(&unpause);
}