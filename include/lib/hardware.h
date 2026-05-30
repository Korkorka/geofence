#pragma once

// UART and MAVLink setup + LED blink
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define LED_DELAY_MS 250

#include <common/mavlink.h>
#include <minimal/mavlink.h>
#include <standard/mavlink.h>

typedef enum{
    IDLE = 0,
    SEND_STREAM_REQ,
    SEND_PAUSE,
    CALCULATE_RETURN,
    SEND_RESUME,
    SEND_UNPAUSE,
    REQUEST_FENCE,
    SETUP_GEOFENCE,
}core0_comms_t;

typedef enum{
    RUNNING = 0,
    PRINT,
    BOOL_INCOMMING, // Reserved for if I decide to delegate the geofence check to Core 1
}core1_comms_t;

typedef enum{
    PAUSE_SENDING = 1,
    RETURN_CALCULATING,
    SENDING_RESUME,
    SENDING_UNPAUSE,
    REQUESTING_MISSION,
    SETTING_UP_GEOFENCE,
}debugging_feedback_t;

int pico_led_init(void);
void pico_set_led(bool);
void send_mav(mavlink_message_t*);