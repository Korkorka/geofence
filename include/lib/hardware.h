// UART and MAVLink setup + LED blink
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define LED_DELAY_MS 250

#include <mavlink/common/mavlink.h>
#include <mavlink/minimal/mavlink.h>
#include <mavlink/standard/mavlink.h>

typedef enum{
    IDLE = 0,
    SEND_STREAM_REQ,
    SEND_PAUSE,
    CALCULATE_RETURN,
    SEND_RESUME,
    SEND_UNPAUSE,
}core0_comms_t;

typedef enum{
    RUNNING = 0,
    PRINT,
    BOOL_INCOMMING, // Reserved for if I decide to delegate the geofence check to Core 1
}core1_comms_t;

int pico_led_init(void);
void pico_set_led(bool);
void send_mav(mavlink_message_t*);
void usart_init(void);