// Pico SDK libraries
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"
#include "hardware/irq.h"
// C common libraries
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
// Mavlink libraries
#include <mavlink/common/mavlink.h>
#include <mavlink/minimal/mavlink.h>
#include <mavlink/standard/mavlink.h>
// Custom Libraries
#include "hardware.h"

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
    uint16_t len = mavlink_msg_to_send_buffer(tx_arr, msg);
    uart_write_blocking(UART_ID, tx_arr, len);
}