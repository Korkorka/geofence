// // Pico SDK libraries
// #include "pico/stdlib.h"
// #include "hardware/gpio.h"
// #include "pico/cyw43_arch.h"
// #include "hardware/watchdog.h"
// #include "pico/multicore.h"
// #include "hardware/irq.h"
// #include "hardware/timer.h"
// // C common libraries
// #include <stdio.h>
// #include <math.h>
// #include <stdlib.h>
// // MAVLink libraries
// #include <mavlink/common/mavlink.h>
// #include <mavlink/minimal/mavlink.h>
// #include <mavlink/standard/mavlink.h>
// // UART and MAVLink setup + LED blink
// #define UART_ID uart0
// #define BAUD_RATE 115200
// #define DATA_BITS 8
// #define STOP_BITS 1
// #define PARITY UART_PARITY_NONE
// #define UART_TX_PIN 0
// #define UART_RX_PIN 1
// #define LED_DELAY_MS 250

// // Definition for safe Geofence offset and calculation for the degree offset, approximates the offset as in cartesians
// #define geof_offset_m 1
// #define geof_prec 9
// const double earth_approx_rad = 6371000;
// const int32_t geof_offset_deg = (int32_t)(((double)(geof_offset_m * 180) / (earth_approx_rad * M_PI)) * 1e7);

// typedef struct{ // Coordinate struct with latt and long expressed multplied by 1e7
//     int32_t delta_lat_e7;
//     int32_t phi_long_e7;
// }coords_t; 

// typedef struct{ // Geofence struct with adaptive sizing
//     uint8_t num_of_waypoints;
//     coords_t* waypoints;
//     int32_t extremas_long[2];
//     int32_t extremas_latt[2];
//     double long_avg; 
//     double latt_avg;
// }geofence_t;

// typedef enum{
//     IDLE = 0,
//     SEND_STREAM_REQ,
//     SEND_PAUSE,
//     SENDING_COORDS,
//     CALCULATE_RETURN,
//     SEND_CORRECTION,
//     SEND_RESUME,
// }core0_comms_t;

// typedef enum{
//     RUNNING = 0,
//     PRINT,
//     BOOL_INCOMMING, // Reserved for if I decide to delegate the geofence check to Core 1
// }core1_comms_t;

// // Global global variables
// // MAVLINK recieves
// mavlink_status_t status;
// mavlink_message_t msg, test;
// mavlink_command_ack_t ack;
// mavlink_heartbeat_t heartbeat;
// mavlink_global_position_int_t position;
// mavlink_attitude_t attitude;
// // MAVLINK commands
// mavlink_message_t request_stream, pause, unpause, correct, stream, correct_resume, pico_heartbeat;
// const int32_t correct_alt = 50;
// const uint8_t system_id = 1, component_id_mc = 200, component_id_fc = 1, chan = MAVLINK_COMM_2, UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
// // UART handling params
// volatile uint8_t byte;

// // Core instruction enums
// core0_comms_t core1_instruction = IDLE; 
// core1_comms_t core0_instruction = RUNNING;

// // Core 1 variables and flags (for safety)
// volatile int32_t lattitude, longitude;
// volatile uint16_t heading;
// volatile float yaw;
// struct repeating_timer timer;
// bool ping = false;
// // Core 0 variables and flags (for safety)
// volatile bool wait = false, first_message = true, calculate = false, correcting = false, send_att = false; 

// // Coords setup + geofence as a global variable
// coords_t coords[] = {{549130910, 97803710}, {549128980, 97807300}, {549127420, 97804640}, {549129970, 97801600}};
// geofence_t geofence = {sizeof(coords)/sizeof(coords_t), NULL, {INT32_MAX, INT32_MIN}, {INT32_MAX, INT32_MIN}, 0, 0};

// // Function prototypes
// int pico_led_init(void);
// void pico_set_led(bool);
// void init(void);
// void send_mav(mavlink_message_t*);
// bool check_geofence(int32_t, int32_t);
// void calculate_return_coords(int32_t, int32_t, uint16_t, float);

// void main(){
//     init();
//     //test coordinates manutally fed
//     int32_t latti = 54912728, longi = 97806245, head, yaw1;
//     bool checking;

//     while(true){
//         pico_set_led(true);
//         sleep_ms(LED_DELAY_MS);
//         checking = check_geofence(latti, longi);
//         printf("%d\n", checking);
//         if(!checking){
//             calculate_return_coords(latti, longi, head, yaw1);
//         }
//         pico_set_led(false);
//         sleep_ms(LED_DELAY_MS);
//     }   

//     free(geofence.waypoints);
// }

// void init(void){
//     stdio_init_all();
//     pico_led_init();

//     geofence.waypoints = malloc(geofence.num_of_waypoints * sizeof(coords_t)); // Allocates pointer memory to the waypoints data in the geofence struct

//     for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){ // Loads up the waypoints into the struct
//         geofence.waypoints[i] = coords[i];
//         geofence.latt_avg += ((double)geofence.waypoints[i].delta_lat_e7 / (double)geofence.num_of_waypoints);
//         geofence.long_avg += ((double)geofence.waypoints[i].phi_long_e7 / (double)geofence.num_of_waypoints);
//     }

//     printf("%f, %f", geofence.latt_avg, geofence.long_avg);

//     for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){ // Corrects the soft geofence for offset which is defined at the beginning
//         if(geofence.waypoints[i].delta_lat_e7 < geofence.latt_avg){
//             geofence.waypoints[i].delta_lat_e7 += geof_offset_deg;
//         }
//         else{
//             geofence.waypoints[i].delta_lat_e7 -= geof_offset_deg;
//         }
//         if(geofence.waypoints[i].phi_long_e7 < geofence.long_avg){
//             geofence.waypoints[i].phi_long_e7 += geof_offset_deg;
//         }
//         else{
//             geofence.waypoints[i].phi_long_e7 -= geof_offset_deg;
//         }
//         if(geofence.extremas_long[0] > geofence.waypoints[i].phi_long_e7){geofence.extremas_long[0] = geofence.waypoints[i].phi_long_e7;}
//         if(geofence.extremas_long[1] < geofence.waypoints[i].phi_long_e7){geofence.extremas_long[1] = geofence.waypoints[i].phi_long_e7;}
//         if(geofence.extremas_latt[0] > geofence.waypoints[i].delta_lat_e7){geofence.extremas_latt[0] = geofence.waypoints[i].delta_lat_e7;}
//         if(geofence.extremas_latt[1] < geofence.waypoints[i].delta_lat_e7){geofence.extremas_latt[1] = geofence.waypoints[i].delta_lat_e7;}

//     }
// }

// // Perform LED initialisation
// int pico_led_init(void){
// #if defined(PICO_DEFAULT_LED_PIN)
//     gpio_init(PICO_DEFAULT_LED_PIN);
//     gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
//     return PICO_OK;
// #elif defined(CYW43_WL_GPIO_LED_PIN)
//     return cyw43_arch_init();
// #endif
// }

// // Turn the led on or off
// void pico_set_led(bool led_on){
// #if defined(PICO_DEFAULT_LED_PIN)
//     gpio_put(PICO_DEFAULT_LED_PIN, led_on);
// #elif defined(CYW43_WL_GPIO_LED_PIN)
//     cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
// #endif
// }

// // Function which checks the coordinate and whether it lies in the defined geofence
// bool check_geofence(int32_t latti, int32_t longi){
      
//     int32_t pla = latti, plo = longi;

//     // Bounding box check
//     // Checks whether the point doesnt lie outside the extremas of the fence making it lie outside by default
//     if(
//     pla < geofence.extremas_latt[0] ||
//     pla > geofence.extremas_latt[1] ||
//     plo < geofence.extremas_long[0] ||
//     plo > geofence.extremas_long[1]
//     ){
//         printf("Bounding box\n");
//         return false;
//     }

//     // Line intercept algorithm
//     // Checks whether the coordinates lie on the longitude range of a given pair of points from the geofence and checks whether a line drawn in the positive lattitude direction intersects
//     // Based on whether 1 or 2 / 0 intersections are found we can know whether we are inside or ourside the fence
//     uint8_t intercepts = 0;
//     for(uint8_t i = 0, j = geofence.num_of_waypoints - 1; i < geofence.num_of_waypoints; j = i++){
//         int32_t point1_latt = geofence.waypoints[i].delta_lat_e7, point1_long = geofence.waypoints[i].phi_long_e7;
//         int32_t point2_latt = geofence.waypoints[j].delta_lat_e7, point2_long = geofence.waypoints[j].phi_long_e7;

//         if(((plo < point1_long) != (plo < point2_long)) && (pla < (int32_t)round(((double)(point1_latt - point2_latt) / (double)(point1_long - point2_long)) * (double)(plo - point1_long) + (double)point1_latt))){intercepts++;}
          
//     }
//     bool check = false;
//     if((intercepts % 2)){check = true;}
//     printf("%d\n", intercepts);
//     return check;
// }


// // Proposed function, to let the drone wait after pause, and calculate some manner of valid return coordinate into the geofence before mission continuation
// void calculate_return_coords(int32_t lat, int32_t lon, uint16_t heading, float yaw) {
//     double min_square_dist = -1.0;
//     int32_t closest_latt = 0;
//     int32_t closest_long = 0;

//     // 1. Calculate Longitude scaling factor based on average latitude (Equirectangular approximation)
//     // cos(lat * pi / 180). We divide by 1e7 because coordinates are scaled.
//     double lat_rad = (geofence.latt_avg / 1e7) * (M_PI / 180.0);
//     double lon_scale = cos(lat_rad);

//     // Convert drone position to scaled Cartesian for accurate math
//     double px = (double)lon * lon_scale;
//     double py = (double)lat;

//     // 2. Find the exact closest point on the polygon perimeter
//     for (uint8_t i = 0, j = geofence.num_of_waypoints - 1; i < geofence.num_of_waypoints; j = i++) {
//         double ax = (double)geofence.waypoints[j].phi_long_e7 * lon_scale;
//         double ay = (double)geofence.waypoints[j].delta_lat_e7;
//         double bx = (double)geofence.waypoints[i].phi_long_e7 * lon_scale;
//         double by = (double)geofence.waypoints[i].delta_lat_e7;

//         // Vector AB and AP
//         double ab_x = bx - ax;
//         double ab_y = by - ay;
//         double ap_x = px - ax;
//         double ap_y = py - ay;

//         // Project P onto line segment AB (find dot product ratio)
//         double ab_squared = (ab_x * ab_x) + (ab_y * ab_y);
//         double t = 0.0;
//         if (ab_squared > 0) {
//             t = ((ap_x * ab_x) + (ap_y * ab_y)) / ab_squared;
//         }

//         // Clamp 't' to [0, 1] to ensure the point stays on the line segment
//         if (t < 0.0) t = 0.0;
//         if (t > 1.0) t = 1.0;

//         // Calculate the closest point C on this segment
//         double cx = ax + (t * ab_x);
//         double cy = ay + (t * ab_y);

//         // Calculate squared distance from drone P to point C
//         double dx = px - cx;
//         double dy = py - cy;
//         double square_dist = (dx * dx) + (dy * dy);

//         if (min_square_dist < 0 || square_dist < min_square_dist) {
//             min_square_dist = square_dist;
//             // Convert back from Cartesian to Lat/Lon e7
//             closest_long = (int32_t)(cx / lon_scale);
//             closest_latt = (int32_t)cy;
//         }
//     }

//     printf("Closest Edge Point: Lat %d, Lon %d\n", closest_latt, closest_long);

//     // 3. Apply safe offset towards the inside of the geofence (Centroid)
//     double cx_to_center = geofence.long_avg - closest_long;
//     double cy_to_center = geofence.latt_avg - closest_latt;
    
//     // Calculate length of the vector to the center
//     double dist_to_center = sqrt((cx_to_center * lon_scale) * (cx_to_center * lon_scale) + (cy_to_center * cy_to_center));

//     int32_t correct_latt = closest_latt;
//     int32_t correct_long = closest_long;

//     if (dist_to_center > 0) {
//         // Normalize the vector and multiply by our desired offset (geof_offset_deg)
//         // Divide lon offset by lon_scale to revert it back to degrees e7
//         double offset_lon = (cx_to_center / dist_to_center) * ((double)geof_offset_deg / lon_scale);
//         double offset_lat = (cy_to_center / dist_to_center) * (double)geof_offset_deg;

//         correct_long = closest_long + (int32_t)offset_lon;
//         correct_latt = closest_latt + (int32_t)offset_lat;
//     }

//     printf("Calculated Return Point (with offset): Lat %d, Lon %d\n", correct_latt, correct_long);
// }



// gemini code 2 so test and if not working use above 
// Pico SDK libraries
// #include "pico/stdlib.h"
// #include "hardware/gpio.h"
// #include "pico/cyw43_arch.h"
// #include "hardware/watchdog.h"
// #include "pico/multicore.h"
// #include "hardware/irq.h"
// #include "hardware/timer.h"
// // C common libraries
// #include <stdio.h>
// #include <math.h>
// #include <stdlib.h>
// // MAVLink libraries
// #include <mavlink/common/mavlink.h>
// #include <mavlink/minimal/mavlink.h>
// #include <mavlink/standard/mavlink.h>
// // UART and MAVLink setup + LED blink
// #define UART_ID uart0
// #define BAUD_RATE 115200
// #define DATA_BITS 8
// #define STOP_BITS 1
// #define PARITY UART_PARITY_NONE
// #define UART_TX_PIN 0
// #define UART_RX_PIN 1
// #define LED_DELAY_MS 250

// // Definition for safe Geofence offset and calculation for the degree offset, approximates the offset as in cartesians
// #define geof_offset_m 1
// #define geof_prec 9
// const double earth_approx_rad = 6371000;
// const int32_t geof_offset_deg = (int32_t)(((double)(geof_offset_m * 180) / (earth_approx_rad * M_PI)) * 1e7);

// typedef struct{ // Coordinate struct with latt and long expressed multplied by 1e7
//     int32_t delta_lat_e7;
//     int32_t phi_long_e7;
// }coords_t; 

// typedef struct{ // Geofence struct with adaptive sizing
//     uint8_t num_of_waypoints;
//     coords_t* waypoints;
//     int32_t extremas_long[2];
//     int32_t extremas_latt[2];
//     double long_avg; 
//     double latt_avg;
// }geofence_t;

// typedef enum{
//     IDLE = 0,
//     SEND_STREAM_REQ,
//     SEND_PAUSE,
//     SENDING_COORDS,
//     CALCULATE_RETURN,
//     SEND_CORRECTION,
//     SEND_RESUME,
// }core0_comms_t;

// typedef enum{
//     RUNNING = 0,
//     PRINT,
//     BOOL_INCOMMING, // Reserved for if I decide to delegate the geofence check to Core 1
// }core1_comms_t;

// // Global global variables
// // MAVLINK recieves
// mavlink_status_t status;
// mavlink_message_t msg, test;
// mavlink_command_ack_t ack;
// mavlink_heartbeat_t heartbeat;
// mavlink_global_position_int_t position;
// mavlink_attitude_t attitude;
// // MAVLINK commands
// mavlink_message_t request_stream, pause, unpause, correct, stream, correct_resume, pico_heartbeat;
// const int32_t correct_alt = 50;
// const uint8_t system_id = 1, component_id_mc = 200, component_id_fc = 1, chan = MAVLINK_COMM_2, UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
// // UART handling params
// volatile uint8_t byte;

// // Core instruction enums
// core0_comms_t core1_instruction = IDLE; 
// core1_comms_t core0_instruction = RUNNING;

// // Core 1 variables and flags (for safety)
// volatile int32_t lattitude, longitude;
// volatile uint16_t heading;
// volatile float yaw;
// struct repeating_timer timer;
// bool ping = false;
// // Core 0 variables and flags (for safety)
// volatile bool wait = false, first_message = true, calculate = false, correcting = false, send_att = false; 

// // Coords setup + geofence as a global variable
// coords_t coords[] = {{549130910, 97803710}, {549128980, 97807300}, {549127420, 97804640}, {549129970, 97801600}};
// geofence_t geofence = {sizeof(coords)/sizeof(coords_t), NULL, {INT32_MAX, INT32_MIN}, {INT32_MAX, INT32_MIN}, 0, 0};

// // Function prototypes
// int pico_led_init(void);
// void pico_set_led(bool);
// void init(void);
// void send_mav(mavlink_message_t*);
// bool check_geofence(int32_t, int32_t);
// void calculate_return_coords(int32_t, int32_t, uint16_t, float);


// // --- MAIN FUNCTION ---
// void main(){
//     init();
//     sleep_ms(3000);
//     // Put your test coordinates here! 
//     int32_t latti = 549128367; 
//     int32_t longi = 97807538; 
//     uint16_t head = 0; 
//     float yaw1 = 0.0f;
    
//     bool has_tested = false; // Flag to prevent terminal spam

//     while(true){
        
//         // This will only run ONCE per reboot/flash
//         if (!has_tested) {
//             printf("\n--- RUNNING GEOFENCE TEST ---\n");
//             // Divide by 1e7 and format to 7 decimal places for easy reading
//             printf("Testing Position: Lat %.7ld, Lon %.7ld\n", latti , longi );
            
//             bool checking = check_geofence(latti, longi);
            
//             if(!checking){
//                 printf("Result: OUTSIDE Geofence. Calculating return...\n");
//                 calculate_return_coords(latti, longi, head, yaw1);
//             } else {
//                 printf("Result: INSIDE Geofence. Safe!\n");
//             }
            
//             printf("--- TEST FINISHED. WAITING... ---\n\n");
            
//             has_tested = true; 
//         }

//         // Just peacefully blink the LED forever
//         pico_set_led(true);
//         sleep_ms(LED_DELAY_MS);
//         pico_set_led(false);
//         sleep_ms(LED_DELAY_MS);
//     }   

//     free(geofence.waypoints);
// }


// void init(void){
//     stdio_init_all();
//     pico_led_init();

//     geofence.waypoints = malloc(geofence.num_of_waypoints * sizeof(coords_t)); // Allocates pointer memory to the waypoints data in the geofence struct

//     for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){ // Loads up the waypoints into the struct
//         geofence.waypoints[i] = coords[i];
//         geofence.latt_avg += ((double)geofence.waypoints[i].delta_lat_e7 / (double)geofence.num_of_waypoints);
//         geofence.long_avg += ((double)geofence.waypoints[i].phi_long_e7 / (double)geofence.num_of_waypoints);
//     }

//     printf("Center Avg: Lat %.7f, Lon %.7f\n", geofence.latt_avg / 1e7, geofence.long_avg / 1e7);

//     for(uint8_t i = 0; i < geofence.num_of_waypoints; i++){ // Corrects the soft geofence for offset which is defined at the beginning
//         if(geofence.waypoints[i].delta_lat_e7 < geofence.latt_avg){
//             geofence.waypoints[i].delta_lat_e7 += geof_offset_deg;
//         }
//         else{
//             geofence.waypoints[i].delta_lat_e7 -= geof_offset_deg;
//         }
//         if(geofence.waypoints[i].phi_long_e7 < geofence.long_avg){
//             geofence.waypoints[i].phi_long_e7 += geof_offset_deg;
//         }
//         else{
//             geofence.waypoints[i].phi_long_e7 -= geof_offset_deg;
//         }
//         if(geofence.extremas_long[0] > geofence.waypoints[i].phi_long_e7){geofence.extremas_long[0] = geofence.waypoints[i].phi_long_e7;}
//         if(geofence.extremas_long[1] < geofence.waypoints[i].phi_long_e7){geofence.extremas_long[1] = geofence.waypoints[i].phi_long_e7;}
//         if(geofence.extremas_latt[0] > geofence.waypoints[i].delta_lat_e7){geofence.extremas_latt[0] = geofence.waypoints[i].delta_lat_e7;}
//         if(geofence.extremas_latt[1] < geofence.waypoints[i].delta_lat_e7){geofence.extremas_latt[1] = geofence.waypoints[i].delta_lat_e7;}

//     }
// }

// // Perform LED initialisation
// int pico_led_init(void){
// #if defined(PICO_DEFAULT_LED_PIN)
//     gpio_init(PICO_DEFAULT_LED_PIN);
//     gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
//     return PICO_OK;
// #elif defined(CYW43_WL_GPIO_LED_PIN)
//     return cyw43_arch_init();
// #endif
// }

// // Turn the led on or off
// void pico_set_led(bool led_on){
// #if defined(PICO_DEFAULT_LED_PIN)
//     gpio_put(PICO_DEFAULT_LED_PIN, led_on);
// #elif defined(CYW43_WL_GPIO_LED_PIN)
//     cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
// #endif
// }

// // Function which checks the coordinate and whether it lies in the defined geofence
// bool check_geofence(int32_t latti, int32_t longi){
      
//     int32_t pla = latti, plo = longi;

//     // Bounding box check
//     // Checks whether the point doesnt lie outside the extremas of the fence making it lie outside by default
//     if(
//     pla < geofence.extremas_latt[0] ||
//     pla > geofence.extremas_latt[1] ||
//     plo < geofence.extremas_long[0] ||
//     plo > geofence.extremas_long[1]
//     ){
//         printf("Failed Bounding Box Check\n");
//         return false;
//     }

//     // Line intercept algorithm
//     // Checks whether the coordinates lie on the longitude range of a given pair of points from the geofence and checks whether a line drawn in the positive lattitude direction intersects
//     // Based on whether 1 or 2 / 0 intersections are found we can know whether we are inside or ourside the fence
//     uint8_t intercepts = 0;
//     for(uint8_t i = 0, j = geofence.num_of_waypoints - 1; i < geofence.num_of_waypoints; j = i++){
//         int32_t point1_latt = geofence.waypoints[i].delta_lat_e7, point1_long = geofence.waypoints[i].phi_long_e7;
//         int32_t point2_latt = geofence.waypoints[j].delta_lat_e7, point2_long = geofence.waypoints[j].phi_long_e7;

//         if(((plo < point1_long) != (plo < point2_long)) && (pla < (int32_t)round(((double)(point1_latt - point2_latt) / (double)(point1_long - point2_long)) * (double)(plo - point1_long) + (double)point1_latt))){intercepts++;}
          
//     }
//     bool check = false;
//     if((intercepts % 2)){check = true;}
//     printf("Line Intercepts: %d\n", intercepts);
//     return check;
// }


// // Proposed function, to let the drone wait after pause, and calculate some manner of valid return coordinate into the geofence before mission continuation
// void calculate_return_coords(int32_t lat, int32_t lon, uint16_t heading, float yaw) {
//     double min_square_dist = -1.0;
//     int32_t closest_latt = 0;
//     int32_t closest_long = 0;

//     // 1. Calculate Longitude scaling factor based on average latitude (Equirectangular approximation)
//     // cos(lat * pi / 180). We divide by 1e7 because coordinates are scaled.
//     double lat_rad = (geofence.latt_avg / 1e7) * (M_PI / 180.0);
//     double lon_scale = cos(lat_rad);

//     // Convert drone position to scaled Cartesian for accurate math
//     double px = (double)lon * lon_scale;
//     double py = (double)lat;

//     // 2. Find the exact closest point on the polygon perimeter
//     for (uint8_t i = 0, j = geofence.num_of_waypoints - 1; i < geofence.num_of_waypoints; j = i++) {
//         double ax = (double)geofence.waypoints[j].phi_long_e7 * lon_scale;
//         double ay = (double)geofence.waypoints[j].delta_lat_e7;
//         double bx = (double)geofence.waypoints[i].phi_long_e7 * lon_scale;
//         double by = (double)geofence.waypoints[i].delta_lat_e7;

//         // Vector AB and AP
//         double ab_x = bx - ax;
//         double ab_y = by - ay;
//         double ap_x = px - ax;
//         double ap_y = py - ay;

//         // Project P onto line segment AB (find dot product ratio)
//         double ab_squared = (ab_x * ab_x) + (ab_y * ab_y);
//         double t = 0.0;
//         if (ab_squared > 0) {
//             t = ((ap_x * ab_x) + (ap_y * ab_y)) / ab_squared;
//         }

//         // Clamp 't' to [0, 1] to ensure the point stays on the line segment
//         if (t < 0.0) t = 0.0;
//         if (t > 1.0) t = 1.0;

//         // Calculate the closest point C on this segment
//         double cx = ax + (t * ab_x);
//         double cy = ay + (t * ab_y);

//         // Calculate squared distance from drone P to point C
//         double dx = px - cx;
//         double dy = py - cy;
//         double square_dist = (dx * dx) + (dy * dy);

//         if (min_square_dist < 0 || square_dist < min_square_dist) {
//             min_square_dist = square_dist;
//             // Convert back from Cartesian to Lat/Lon e7
//             closest_long = (int32_t)(cx / lon_scale);
//             closest_latt = (int32_t)cy;
//         }
//     }

//     printf("Closest Edge Point: Lat %.7f, Lon %.7f\n", closest_latt / 1e7, closest_long / 1e7);

//     // 3. Apply safe offset towards the inside of the geofence (Centroid)
//     double cx_to_center = geofence.long_avg - closest_long;
//     double cy_to_center = geofence.latt_avg - closest_latt;
    
//     // Calculate length of the vector to the center
//     double dist_to_center = sqrt((cx_to_center * lon_scale) * (cx_to_center * lon_scale) + (cy_to_center * cy_to_center));

//     int32_t correct_latt = closest_latt;
//     int32_t correct_long = closest_long;

//     if (dist_to_center > 0) {
//         // Normalize the vector and multiply by our desired offset (geof_offset_deg)
//         // Divide lon offset by lon_scale to revert it back to degrees e7
//         double offset_lon = (cx_to_center / dist_to_center) * ((double)geof_offset_deg / lon_scale);
//         double offset_lat = (cy_to_center / dist_to_center) * (double)geof_offset_deg;

//         correct_long = closest_long + (int32_t)offset_lon;
//         correct_latt = closest_latt + (int32_t)offset_lat;
//     }

//     printf("Calculated Return Point (with offset): Lat %.7f, Lon %.7f\n", correct_latt / 1e7, correct_long / 1e7);
// } 
//above works gooed enough, needs more testing


// //below code 3 with more slight improvements , lets see how its 
// #include "pico/stdlib.h"
// #include "hardware/gpio.h"
// #include "pico/cyw43_arch.h"

// #include <stdio.h>
// #include <math.h>
// #include <stdlib.h>
// #include <stdint.h>
// #include <stdbool.h>

// #define LED_DELAY_MS 250

// #define GEOF_OFFSET_M 1.0
// #define EARTH_RADIUS_M 6371000.0

// typedef struct {
//     int32_t lat_e7;
//     int32_t lon_e7;
// } coords_t;

// typedef struct {
//     uint8_t num_points;
//     coords_t *points;

//     int32_t min_lat;
//     int32_t max_lat;

//     int32_t min_lon;
//     int32_t max_lon;

//     double center_lat;
//     double center_lon;

// } geofence_t;

// const int32_t geof_offset_e7 =
//     (int32_t)((GEOF_OFFSET_M * 180.0) /
//     (EARTH_RADIUS_M * M_PI) * 1e7);

// coords_t raw_coords[] = {
//     {549130910, 97803710},
//     {549128980, 97807300},
//     {549127420, 97804640},
//     {549129970, 97801600}
// };

// geofence_t geofence = {
//     sizeof(raw_coords)/sizeof(coords_t),
//     NULL,
//     INT32_MAX,
//     INT32_MIN,
//     INT32_MAX,
//     INT32_MIN,
//     0,
//     0
// };

// int pico_led_init(void);
// void pico_set_led(bool led_on);

// void init_geofence(void);

// bool check_geofence(int32_t lat, int32_t lon);

// void calculate_return_coords(
//     int32_t lat,
//     int32_t lon
// );

// int main() {

//     stdio_init_all();

//     pico_led_init();

//     init_geofence();

//     sleep_ms(3000);

//     // VALID degE7 coordinates
//     int32_t test_lat = 549128367;
//     int32_t test_lon = 97807538;

//     printf("\n--- RUNNING GEOFENCE TEST ---\n");

//     printf(
//         "Testing Position: Lat %.7f Lon %.7f\n",
//         (double)test_lat / 1e7,
//         (double)test_lon / 1e7
//     );

//     bool inside = check_geofence(
//         test_lat,
//         test_lon
//     );

//     if (inside) {

//         printf("Result: INSIDE geofence\n");

//     } else {

//         printf("Result: OUTSIDE geofence\n");

//         calculate_return_coords(
//             test_lat,
//             test_lon
//         );
//     }

//     while (true) {

//         pico_set_led(true);
//         sleep_ms(LED_DELAY_MS);

//         pico_set_led(false);
//         sleep_ms(LED_DELAY_MS);
//     }
// }

// void init_geofence(void) {

//     geofence.points =
//         malloc(sizeof(raw_coords));

//     if (!geofence.points) {

//         printf("Malloc failed\n");

//         while (true);
//     }

//     for (uint8_t i = 0;
//          i < geofence.num_points;
//          i++) {

//         geofence.points[i] = raw_coords[i];

//         geofence.center_lat +=
//             (double)raw_coords[i].lat_e7 /
//             geofence.num_points;

//         geofence.center_lon +=
//             (double)raw_coords[i].lon_e7 /
//             geofence.num_points;
//     }

//     printf(
//         "Fence Center: %.7f %.7f\n",
//         geofence.center_lat / 1e7,
//         geofence.center_lon / 1e7
//     );

//     for (uint8_t i = 0;
//          i < geofence.num_points;
//          i++) {

//         coords_t *p = &geofence.points[i];

//         // inward soft offset
//         if (p->lat_e7 < geofence.center_lat)
//             p->lat_e7 += geof_offset_e7;
//         else
//             p->lat_e7 -= geof_offset_e7;

//         if (p->lon_e7 < geofence.center_lon)
//             p->lon_e7 += geof_offset_e7;
//         else
//             p->lon_e7 -= geof_offset_e7;

//         // bounding box
//         if (p->lat_e7 < geofence.min_lat)
//             geofence.min_lat = p->lat_e7;

//         if (p->lat_e7 > geofence.max_lat)
//             geofence.max_lat = p->lat_e7;

//         if (p->lon_e7 < geofence.min_lon)
//             geofence.min_lon = p->lon_e7;

//         if (p->lon_e7 > geofence.max_lon)
//             geofence.max_lon = p->lon_e7;
//     }
// }

// bool check_geofence(
//     int32_t lat,
//     int32_t lon
// ) {

//     // bounding reject
//     if (
//         lat < geofence.min_lat ||
//         lat > geofence.max_lat ||
//         lon < geofence.min_lon ||
//         lon > geofence.max_lon
//     ) {

//         printf("Failed bounding box check\n");

//         return false;
//     }

//     bool inside = false;

//     for (
//         uint8_t i = 0,
//         j = geofence.num_points - 1;

//         i < geofence.num_points;

//         j = i++
//     ) {

//         double xi = geofence.points[i].lon_e7;
//         double yi = geofence.points[i].lat_e7;

//         double xj = geofence.points[j].lon_e7;
//         double yj = geofence.points[j].lat_e7;

//         bool intersect =
//             ((yi > lat) != (yj > lat)) &&
//             (lon <
//             (xj - xi) *
//             (lat - yi) /
//             (yj - yi) +
//             xi);

//         if (intersect)
//             inside = !inside;
//     }

//     printf(
//         "Polygon Result: %s\n",
//         inside ? "INSIDE" : "OUTSIDE"
//     );

//     return inside;
// }

// void calculate_return_coords(
//     int32_t lat,
//     int32_t lon
// ) {

//     double lat_rad =
//         (geofence.center_lat / 1e7) *
//         (M_PI / 180.0);

//     double lon_scale = cos(lat_rad);

//     double px = lon * lon_scale;
//     double py = lat;

//     double best_dist = -1;

//     int32_t best_lat = 0;
//     int32_t best_lon = 0;

//     for (
//         uint8_t i = 0,
//         j = geofence.num_points - 1;

//         i < geofence.num_points;

//         j = i++
//     ) {

//         double ax =
//             geofence.points[j].lon_e7 *
//             lon_scale;

//         double ay =
//             geofence.points[j].lat_e7;

//         double bx =
//             geofence.points[i].lon_e7 *
//             lon_scale;

//         double by =
//             geofence.points[i].lat_e7;

//         double abx = bx - ax;
//         double aby = by - ay;

//         double apx = px - ax;
//         double apy = py - ay;

//         double ab_len_sq =
//             abx * abx + aby * aby;

//         double t = 0.0;

//         if (ab_len_sq > 0.0) {

//             t =
//                 (apx * abx + apy * aby)
//                 / ab_len_sq;
//         }

//         if (t < 0.0) t = 0.0;
//         if (t > 1.0) t = 1.0;

//         double cx = ax + t * abx;
//         double cy = ay + t * aby;

//         double dx = px - cx;
//         double dy = py - cy;

//         double dist_sq =
//             dx * dx + dy * dy;

//         if (
//             best_dist < 0 ||
//             dist_sq < best_dist
//         ) {

//             best_dist = dist_sq;

//             best_lon =
//                 (int32_t)(cx / lon_scale);

//             best_lat =
//                 (int32_t)cy;
//         }
//     }

//     printf(
//         "Closest Edge Point: %.7f %.7f\n",
//         (double)best_lat / 1e7,
//         (double)best_lon / 1e7
//     );

//     // inward offset
//     double vx =
//         geofence.center_lon - best_lon;

//     double vy =
//         geofence.center_lat - best_lat;

//     double len =
//         sqrt(
//             (vx * lon_scale) *
//             (vx * lon_scale) +
//             vy * vy
//         );

//     int32_t safe_lat = best_lat;
//     int32_t safe_lon = best_lon;

//     if (len > 0.0) {

//         double off_lon =
//             (vx / len) *
//             ((double)geof_offset_e7 /
//             lon_scale);

//         double off_lat =
//             (vy / len) *
//             geof_offset_e7;

//         safe_lon += (int32_t)off_lon;
//         safe_lat += (int32_t)off_lat;
//     }

//     printf(
//         "Safe Return Point: %.7f %.7f\n",
//         (double)safe_lat / 1e7,
//         (double)safe_lon / 1e7
//     );
// }

// int pico_led_init(void) {

// #if defined(PICO_DEFAULT_LED_PIN)

//     gpio_init(PICO_DEFAULT_LED_PIN);

//     gpio_set_dir(
//         PICO_DEFAULT_LED_PIN,
//         GPIO_OUT
//     );

//     return PICO_OK;

// #elif defined(CYW43_WL_GPIO_LED_PIN)

//     return cyw43_arch_init();

// #endif
// }

// void pico_set_led(bool led_on) {

// #if defined(PICO_DEFAULT_LED_PIN)

//     gpio_put(
//         PICO_DEFAULT_LED_PIN,
//         led_on
//     );

// #elif defined(CYW43_WL_GPIO_LED_PIN)

//     cyw43_arch_gpio_put(
//         CYW43_WL_GPIO_LED_PIN,
//         led_on
//     );

// #endif
// } works ok



//below code 3 with more slight improvements , lets see how its 
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define LED_DELAY_MS 250

#define GEOF_OFFSET_M 1.0
#define EARTH_RADIUS_M 6371000.0

typedef struct {
    int32_t lat_e7;
    int32_t lon_e7;
} coords_t;

typedef struct {
    uint8_t num_points;
    coords_t *points;

    int32_t min_lat;
    int32_t max_lat;

    int32_t min_lon;
    int32_t max_lon;

    double center_lat;
    double center_lon;

} geofence_t;

const int32_t geof_offset_e7 =
    (int32_t)((GEOF_OFFSET_M * 180.0) /
    (EARTH_RADIUS_M * M_PI) * 1e7);

coords_t raw_coords[] = {
    {549130910, 97803710},
    {549128980, 97807300},
    {549127420, 97804640},
    {549129970, 97801600}
};

geofence_t geofence = {
    sizeof(raw_coords)/sizeof(coords_t),
    NULL,
    INT32_MAX,
    INT32_MIN,
    INT32_MAX,
    INT32_MIN,
    0,
    0
};

int pico_led_init(void);
void pico_set_led(bool led_on);

void init_geofence(void);

bool check_geofence(int32_t lat, int32_t lon);

void calculate_return_coords(
    int32_t lat,
    int32_t lon
);

int main() {

    stdio_init_all();

    pico_led_init();

    init_geofence();

    // Give time for USB serial to connect to the terminal
    sleep_ms(3000); 

    // Array of multiple test coordinates to evaluate
coords_t test_points[] = {
    {549128974, 97807359},
    {549128926, 97807133},
    {549128224, 97806225},
    {549127525, 97805424},
    {549129719, 97806415},
    {549130828, 97806081},
    {549130273, 97805024},
    {549130103, 97804815},
    {549130222, 97804914},
    {549130850, 97803710},
    {549130711, 97803260},
    {549130186, 97801843},
    {549130625, 97802236},
    {549128559, 97802413},
    {549129284, 97804378},
    {549127369, 97803493},
    {549129836, 97800852},
    {549129726, 97802045},
    {549129711, 97802096},
    {549129647, 97802074},
    {549129287, 97802423},
    {549129177, 97802532},
    {549129243, 97802542}
};

    int num_tests = sizeof(test_points) / sizeof(test_points[0]);

    printf("\n--- RUNNING MULTIPLE GEOFENCE TESTS ---\n");

    for (int i = 0; i < num_tests; i++) {
        
        int32_t test_lat = test_points[i].lat_e7;
        int32_t test_lon = test_points[i].lon_e7;

        printf("\n========================================\n");
        printf("Test %d - Testing Position: Lat %.7f Lon %.7f\n",
               i + 1,
               (double)test_lat / 1e7,
               (double)test_lon / 1e7);

        bool inside = check_geofence(
            test_lat,
            test_lon
        );

        if (inside) {

            printf("Result: INSIDE geofence\n");

        } else {

            printf("Result: OUTSIDE geofence\n");

            calculate_return_coords(
                test_lat,
                test_lon
            );
        }
        
        // Brief pause between calculations
        sleep_ms(500);
    }
    
    printf("\n========================================\n");
    printf("Tests complete. Entering LED blink loop...\n");

    // Infinite loop for LED indication
    while (true) {

        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);

        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
    }
}

void init_geofence(void) {

    geofence.points =
        malloc(sizeof(raw_coords));

    if (!geofence.points) {

        printf("Malloc failed\n");

        while (true);
    }

    for (uint8_t i = 0;
         i < geofence.num_points;
         i++) {

        geofence.points[i] = raw_coords[i];

        geofence.center_lat +=
            (double)raw_coords[i].lat_e7 /
            geofence.num_points;

        geofence.center_lon +=
            (double)raw_coords[i].lon_e7 /
            geofence.num_points;
    }

    printf(
        "Fence Center: %.7f %.7f\n",
        geofence.center_lat / 1e7,
        geofence.center_lon / 1e7
    );

    for (uint8_t i = 0;
         i < geofence.num_points;
         i++) {

        coords_t *p = &geofence.points[i];

        // inward soft offset
        if (p->lat_e7 < geofence.center_lat)
            p->lat_e7 += geof_offset_e7;
        else
            p->lat_e7 -= geof_offset_e7;

        if (p->lon_e7 < geofence.center_lon)
            p->lon_e7 += geof_offset_e7;
        else
            p->lon_e7 -= geof_offset_e7;

        // bounding box
        if (p->lat_e7 < geofence.min_lat)
            geofence.min_lat = p->lat_e7;

        if (p->lat_e7 > geofence.max_lat)
            geofence.max_lat = p->lat_e7;

        if (p->lon_e7 < geofence.min_lon)
            geofence.min_lon = p->lon_e7;

        if (p->lon_e7 > geofence.max_lon)
            geofence.max_lon = p->lon_e7;
    }
}

bool check_geofence(
    int32_t lat,
    int32_t lon
) {

    // bounding reject
    if (
        lat < geofence.min_lat ||
        lat > geofence.max_lat ||
        lon < geofence.min_lon ||
        lon > geofence.max_lon
    ) {

        printf("Failed bounding box check\n");

        return false;
    }

    bool inside = false;

    for (
        uint8_t i = 0,
        j = geofence.num_points - 1;

        i < geofence.num_points;

        j = i++
    ) {

        double xi = geofence.points[i].lon_e7;
        double yi = geofence.points[i].lat_e7;

        double xj = geofence.points[j].lon_e7;
        double yj = geofence.points[j].lat_e7;

        bool intersect =
            ((yi > lat) != (yj > lat)) &&
            (lon <
            (xj - xi) *
            (lat - yi) /
            (yj - yi) +
            xi);

        if (intersect)
            inside = !inside;
    }

    printf(
        "Polygon Result: %s\n",
        inside ? "INSIDE" : "OUTSIDE"
    );

    return inside;
}

void calculate_return_coords(
    int32_t lat,
    int32_t lon
) {

    double lat_rad =
        (geofence.center_lat / 1e7) *
        (M_PI / 180.0);

    double lon_scale = cos(lat_rad);

    double px = lon * lon_scale;
    double py = lat;

    double best_dist = -1;

    int32_t best_lat = 0;
    int32_t best_lon = 0;

    for (
        uint8_t i = 0,
        j = geofence.num_points - 1;

        i < geofence.num_points;

        j = i++
    ) {

        double ax =
            geofence.points[j].lon_e7 *
            lon_scale;

        double ay =
            geofence.points[j].lat_e7;

        double bx =
            geofence.points[i].lon_e7 *
            lon_scale;

        double by =
            geofence.points[i].lat_e7;

        double abx = bx - ax;
        double aby = by - ay;

        double apx = px - ax;
        double apy = py - ay;

        double ab_len_sq =
            abx * abx + aby * aby;

        double t = 0.0;

        if (ab_len_sq > 0.0) {

            t =
                (apx * abx + apy * aby)
                / ab_len_sq;
        }

        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;

        double cx = ax + t * abx;
        double cy = ay + t * aby;

        double dx = px - cx;
        double dy = py - cy;

        double dist_sq =
            dx * dx + dy * dy;

        if (
            best_dist < 0 ||
            dist_sq < best_dist
        ) {

            best_dist = dist_sq;

            best_lon =
                (int32_t)(cx / lon_scale);

            best_lat =
                (int32_t)cy;
        }
    }

    printf(
        "Closest Edge Point: %.7f %.7f\n",
        (double)best_lat / 1e7,
        (double)best_lon / 1e7
    );

    // inward offset
    double vx =
        geofence.center_lon - best_lon;

    double vy =
        geofence.center_lat - best_lat;

    double len =
        sqrt(
            (vx * lon_scale) *
            (vx * lon_scale) +
            vy * vy
        );

    int32_t safe_lat = best_lat;
    int32_t safe_lon = best_lon;

    if (len > 0.0) {

        double off_lon =
            (vx / len) *
            ((double)geof_offset_e7 /
            lon_scale);

        double off_lat =
            (vy / len) *
            geof_offset_e7;

        safe_lon += (int32_t)off_lon;
        safe_lat += (int32_t)off_lat;
    }

    printf(
        "Safe Return Point: %.7f %.7f\n",
        (double)safe_lat / 1e7,
        (double)safe_lon / 1e7
    );
}

int pico_led_init(void) {

#if defined(PICO_DEFAULT_LED_PIN)

    gpio_init(PICO_DEFAULT_LED_PIN);

    gpio_set_dir(
        PICO_DEFAULT_LED_PIN,
        GPIO_OUT
    );

    return PICO_OK;

#elif defined(CYW43_WL_GPIO_LED_PIN)

    return cyw43_arch_init();

#endif
}

void pico_set_led(bool led_on) {

#if defined(PICO_DEFAULT_LED_PIN)

    gpio_put(
        PICO_DEFAULT_LED_PIN,
        led_on
    );

#elif defined(CYW43_WL_GPIO_LED_PIN)

    cyw43_arch_gpio_put(
        CYW43_WL_GPIO_LED_PIN,
        led_on
    );

#endif
}