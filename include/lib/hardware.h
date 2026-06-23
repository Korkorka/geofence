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

enum PX4_CUSTOM_MAIN_MODE {
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
    PX4_CUSTOM_MAIN_MODE_ALTCTL = 2,
    PX4_CUSTOM_MAIN_MODE_POSCTL = 3,
    PX4_CUSTOM_MAIN_MODE_AUTO = 4,
    PX4_CUSTOM_MAIN_MODE_ACRO = 5,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6,
    PX4_CUSTOM_MAIN_MODE_STABILIZED = 7,
    PX4_CUSTOM_MAIN_MODE_RATTITUDE_LEGACY = 8,
    PX4_CUSTOM_MAIN_MODE_SIMPLE = 9,
};

enum PX4_CUSTOM_SUB_MODE_AUTO {
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3,
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6,
    PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND = 8,
};


enum PX4_CUSTOM_SUB_MODE_POSCTL {
	PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL = 0,
	PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT,
	PX4_CUSTOM_SUB_MODE_POSCTL_SLOW
};

int pico_led_init(void);
void pico_set_led(bool);
void send_mav(mavlink_message_t*);