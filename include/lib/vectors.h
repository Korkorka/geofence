#pragma once

// C common libraries
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
// Pico SDK libraries
#include "pico/stdlib.h"

// Definition for safe Geofence offset and calculation for the degree offset, approximates the offset as in cartesians
#define INIT_GEOF_OFFSET_M 2.0
#define GEOF_OFFSET_M 1.0
#define EARTH_RADIUS_M 6371000.0
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

typedef struct{ // Coordinate struct with latt and long expressed multplied by 1e7
    int32_t latt_e7;
    int32_t long_e7;
    float yaw;
}coords_t; 

typedef struct{ // Geofence struct with adaptive sizing
    uint8_t num_of_waypoints;
    coords_t* waypoints;
    int32_t extremas_long[2];
    int32_t extremas_latt[2];
    double long_avg; 
    double latt_avg;
}geofence_t;

typedef struct{
    double x;
    double y;
    double z;
}vect3D_t;

double distance(coords_t, coords_t);
double dot(vect3D_t, vect3D_t);
double project(vect3D_t, vect3D_t);
double vect_len(vect3D_t);
double vect_angle(vect3D_t, vect3D_t);
vect3D_t spherical_to_euclid(coords_t);
vect3D_t vect_subtract(vect3D_t, vect3D_t);
vect3D_t vect_add(vect3D_t, vect3D_t);
void vect_multiply(vect3D_t *, double);
void vect_normalize(vect3D_t *);
coords_t euclid_to_spherical(vect3D_t);
vect3D_t cross_prod(vect3D_t, vect3D_t);