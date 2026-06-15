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
#include "coords.h"

// Global variables
// MAVLINK incoming
mavlink_status_t status;
mavlink_message_t msg;
// mavlink_attitude_t attitude;
mavlink_command_ack_t ack;
mavlink_global_position_int_t position;
mavlink_mission_count_t mission_count;
mavlink_mission_item_int_t mission_item;
// MAVLINK outgoing
mavlink_message_t request_stream, pause, unpause, correct, correct_resume, pico_heartbeat, change_yaw, mission_list_req;
const int32_t correct_alt = 50;
const uint8_t system_id = 1, component_id_mc = 190, component_id_fc = 1, chan = MAVLINK_COMM_2, UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
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
coords_t correct_coords;
int32_t alt_check;
// Core 0 variables and flags (for safety)
volatile bool wait = false, first_message = true, calculate = false, correcting = false; 

// Coords setup + geofence as a global variable
coords_t * mission_coords;
coords_t geofence_coords[] = {{549227908, 98168756}, {549231019, 98168033}, {549230972, 98165858}, {549227828, 98166772}};
geofence_t geofence = {sizeof(geofence_coords)/sizeof(coords_t), NULL, {INT32_MAX, INT32_MIN}, {INT32_MAX, INT32_MIN}, 0, 0};

// Function prototypes
void init(void);
void update_system(void); 
void request_mission(uint16_t);

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
    return true;
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

                    if(!(check_geofence(&geofence, position)) && !correcting){
                        multicore_fifo_push_blocking(SEND_PAUSE);
                        calculate = true;
                        correcting = true;
                        // printf("\n\n\n\n\n\n\n\n\nOUTSIDE\nOUTSIDE\nOUTSIDE\nOUTSIDE\n\n\n\n\n\n\n\n\n\n\n"); 
                    }
                    // printf("\nPosition recieved, %d, %d, %u\n", position.lat, position.lon, position.hdg);
                    break;
 
                case(MAVLINK_MSG_ID_MISSION_COUNT):
                    mavlink_msg_mission_count_decode(&msg, &mission_count);
                    watchdog_update();
                    multicore_fifo_push_blocking(REQUEST_FENCE);
                    multicore_fifo_push_blocking(mission_count.count);
                break;

                // case(MAVLINK_MSG_ID_ATTITUDE):
                //     mavlink_msg_attitude_decode(&msg, &attitude); 
                //     watchdog_update();
                // break;

                case(MAVLINK_MSG_ID_MISSION_ITEM_INT):
                    mavlink_msg_mission_item_int_decode(&msg, &mission_item);
                    // printf("%d\n", mission_item.frame);
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
                        printf("\nPause recieved, %d\n", ack.result);
                            // if(ack.result == MAV_RESULT_ACCEPTED){
                            // }
                            break;

                        case(MAV_CMD_DO_REPOSITION):
                        printf("\nReposition acknowledged, %d\n", ack.result);
                            if(ack.result == MAV_RESULT_ACCEPTED && wait == true){
                                multicore_fifo_push_blocking(SEND_RESUME);
                                correcting = false;
                                wait = false;
                            }
                            break;
                    
                    }
            break;
            }
        }
    }
    watchdog_update();
}

// mavlink_global_position_int_t test_position = {
//     .lat = 549129094, 
//     .lon = 97784961,
// };

// bool test_fired = false;

void main(){
    init();

    while(true){
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        watchdog_update();
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
        watchdog_update();
        // if(calculate){
        //     multicore_fifo_push_blocking(CALCULATE_RETURN);
        //     multicore_fifo_push_blocking(position.lat);
        //     multicore_fifo_push_blocking(position.lon);
        //     calculate = false;
        // }
        sleep_ms(5000);
        multicore_fifo_push_blocking(CALCULATE_RETURN);
        // if(!test_fired){
        //     multicore_fifo_push_blocking(SEND_PAUSE);
        //     calculate = true;
        //     correcting = true;
        //     test_fired = true; 
        // }
    }   

    free(geofence.waypoints);
}

void core1_entry(){
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_FIFO);
    irq_set_enabled(SIO_IRQ_PROC1, true);

    add_repeating_timer_ms(1000, alarm_callback, NULL, &timer);

    while(true){
        switch(core1_instruction){
            case(SEND_STREAM_REQ):
            core1_instruction = IDLE;
            send_mav(&request_stream);
            send_mav(&mission_list_req);
            break;

            case(SEND_PAUSE):
            alt_check = PAUSE_SENDING;
            core1_instruction = IDLE;
            send_mav(&pause);
            break;

            case(CALCULATE_RETURN):
            alt_check = RETURN_CALCULATING;
            core1_instruction = IDLE;
            correct_coords = calculate_return(&geofence, lattitude, longitude);
            mavlink_command_long_pack(system_id, component_id_mc, &correct, system_id, component_id_fc, MAV_CMD_DO_REPOSITION, (uint8_t)(0), (float)(-1), (float)(MAV_DO_REPOSITION_FLAGS_CHANGE_MODE), (float)(0), (float)(NAN), (float)(correct_coords.latt_e7), (float)(correct_coords.long_e7), (float)(correct_alt));
            mavlink_msg_command_long_pack(system_id, component_id_mc, &change_yaw, system_id, component_id_fc, MAV_CMD_DO_REPOSITION, (uint8_t)(0), (float)(-1), (float)(MAV_DO_REPOSITION_FLAGS_CHANGE_MODE), (float)(0), (float)(correct_coords.yaw), (float)(lattitude), (float)(longitude), (float)(correct_alt));
            send_mav(&change_yaw);
            send_mav(&unpause);
            send_mav(&correct);
            change = true; 
            break;

            case(SEND_RESUME):
            alt_check = SENDING_RESUME;
            core1_instruction = IDLE;
            send_mav(&correct_resume);
            break;

            case(SEND_UNPAUSE):
            alt_check = SENDING_UNPAUSE;
            core1_instruction = IDLE;
            send_mav(&unpause);
            send_mav(&correct);
            break;

            case(REQUEST_FENCE):
            alt_check = REQUESTING_MISSION;
            core1_instruction = IDLE;
            break;

            case(SETUP_GEOFENCE):
            alt_check = SETTING_UP_GEOFENCE;
            core1_instruction = IDLE;
            // geofence_setup(&geofence);
            break;
        }
        if(change){
            update_system();
        }
        if(ping){
            send_mav(&pico_heartbeat);
            send_mav(&sys_status);
            ping = false;
        }
    }
}

void init(void){
    stdio_init_all();
    pico_led_init();

    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC0, core0_FIFO);
    irq_set_enabled(SIO_IRQ_PROC0, true);

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
    mavlink_msg_heartbeat_pack( system_id, component_id_mc, &pico_heartbeat, MAV_TYPE_ONBOARD_CONTROLLER, (uint8_t)MAV_AUTOPILOT_INVALID, (uint8_t)MAV_MODE_FLAG_AUTO_ENABLED, (uint32_t)0, (uint8_t)MAV_STATE_ACTIVE);
    mavlink_msg_command_long_pack(system_id, component_id_mc, &pause, system_id, component_id_fc, MAV_CMD_DO_PAUSE_CONTINUE, (uint8_t)(0), (float)(MAV_BOOL_FALSE), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0)); // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE 
    mavlink_msg_command_long_pack(system_id, component_id_mc, &unpause, system_id, component_id_fc, MAV_CMD_DO_PAUSE_CONTINUE, (uint8_t)(0), (float)(MAV_BOOL_TRUE), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0), (float)(0)); 
    mavlink_msg_mission_request_list_pack(system_id, component_id_mc, &mission_list_req, system_id, component_id_fc, MAV_MISSION_TYPE_FENCE);
    mavlink_msg_global_position_int_pack(system_id, component_id_mc, &sys_status, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    geofence_setup(geofence_coords, &geofence, geofence.num_of_waypoints);
    // sleep_ms(5000);
    // for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){
    //     printf("\nGeofence coord num: %d, lat: %d long: %d\n", i, geofence.waypoints[i].latt_e7, geofence.waypoints[i].long_e7);
    // }

    // watchdog_enable(1000, 1);
    multicore_launch_core1(core1_entry); // Launches the second core with its main function
    watchdog_update();
}

void update_system(void){
    mavlink_msg_global_position_int_pack(system_id, component_id_mc, &sys_status, 0, correct_coords.latt_e7, correct_coords.long_e7, correcting, 0, 0, 0, 0, (uint16_t)correct_coords.yaw);
    change = false;
}

void request_mission(uint16_t num){
    mavlink_mission_request_int_t mission_item_request;
    mission_coords = malloc(num * sizeof(coords_t));

    // for(uint16_t i = 0; i < num; i++){
    //     mavlink_msg_mission_request_int_pack(system_id, component_id_mc, &mission_item_request, system_id, component_id_fc, i, MAV_MISSION_TYPE_FENCE);
    //     send_mav(&mission_item_request);
    // }
}