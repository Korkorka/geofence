// Custom Libraries
#include "vectors.h"
#include "hardware.h"
// MAVLink libraries
#include <common/mavlink.h>
#include <minimal/mavlink.h>
#include <standard/mavlink.h>
// C common libraries
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
// Pico SDK libraries
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

void geofence_setup(coords_t* mission_coords, geofence_t* geofence, uint8_t num_of_waypoints){
    geofence->waypoints = malloc(num_of_waypoints * sizeof(coords_t)); // Allocates pointer memory to the waypoints data in the geofence struct

    for(uint8_t i = 0; i < geofence->num_of_waypoints; i++){ // Loads up the waypoints into the struct
            geofence->waypoints[i] = mission_coords[i];
            geofence->latt_avg += ((double)geofence->waypoints[i].latt_e7 / (double)geofence->num_of_waypoints);
            geofence->long_avg += ((double)geofence->waypoints[i].long_e7 / (double)geofence->num_of_waypoints);
        }

        for(uint8_t i = 0; i < geofence->num_of_waypoints; i++){ // Corrects the soft geofence for offset which is defined at the beginning
            coords_t coords = {geofence->waypoints[i].latt_e7, geofence->waypoints[i].long_e7}, centre = {geofence->latt_avg, geofence->long_avg};
            vect3D_t P, PC, C;

            P = spherical_to_euclid(coords);
            C = spherical_to_euclid(centre);
            PC = vect_subtract(C, P);
            vect_normalize(&PC);
            vect_multiply(&PC, INIT_GEOF_OFFSET_M);
            P = vect_add(P, PC);
            geofence->waypoints[i] = euclid_to_spherical(P);

            if(geofence->extremas_long[0] > geofence->waypoints[i].long_e7){geofence->extremas_long[0] = geofence->waypoints[i].long_e7;}
            if(geofence->extremas_long[1] < geofence->waypoints[i].long_e7){geofence->extremas_long[1] = geofence->waypoints[i].long_e7;}
            if(geofence->extremas_latt[0] > geofence->waypoints[i].latt_e7){geofence->extremas_latt[0] = geofence->waypoints[i].latt_e7;}
            if(geofence->extremas_latt[1] < geofence->waypoints[i].latt_e7){geofence->extremas_latt[1] = geofence->waypoints[i].latt_e7;}
        }

}

// Function which checks the coordinate and whether it lies in the defined geofence
bool check_geofence(geofence_t* geofence, mavlink_global_position_int_t pos){

    bool inside = false;

    // Bounding box check
    // Checks whether the point doesnt lie outside the extremas of the fence making it lie outside by default
    if(pos.lat < geofence->extremas_latt[0] || pos.lat >  geofence->extremas_latt[1] || pos.lon <  geofence->extremas_long[0] || pos.lon > geofence->extremas_long[1]){
        return inside;
    }

    // Line intercept algorithm 
    // Checks whether the coordinates lie on the longitude range of a given pair of points from the geofence and checks whether a line drawn in the positive lattitude direction intersects
    // Based on whether 1 or 2 / 0 intersections are found we can know whether we are inside or ourside the fence
    for(uint8_t i = 0, j = geofence->num_of_waypoints - 1; i < geofence->num_of_waypoints; j = i++){
        if(((pos.lon < geofence->waypoints[i].long_e7) != (pos.lon < geofence->waypoints[j].long_e7)) && (pos.lat < (int32_t)round(((double)(geofence->waypoints[i].latt_e7 - geofence->waypoints[j].latt_e7) / (double)(geofence->waypoints[i].long_e7 - geofence->waypoints[j].long_e7)) * (double)(pos.lon - geofence->waypoints[i].long_e7) + (double)geofence->waypoints[i].latt_e7))){
            inside = !inside;
        }
    
    }
    return inside;
}


// Proposed function, to let the drone wait after pause, and calculate some manner of valid return coordinate into the geofence before mission continuation
coords_t calculate_return(geofence_t* geofence, int32_t lat, int32_t lon){
    double min_dist = UINT64_MAX, dist, proj_const;
    coords_t point = {lat, lon}, centre = {geofence->latt_avg, geofence->long_avg}, return_coords, return_coords_offset, correct_coords, correct_offset;
    vect3D_t A, B, P, AB, AP, C, EC, E, E_corr;

    for(uint8_t i = 0, j = geofence->num_of_waypoints - 1; i < geofence->num_of_waypoints; j = i++){
        A = spherical_to_euclid(geofence->waypoints[j]);
        B = spherical_to_euclid(geofence->waypoints[i]);
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
    coords_t correct = {correct_offset.latt_e7, correct_offset.long_e7, RAD2DEG((float)atan2(dot(PE, East), dot(PE, North)))};

    return correct;
}