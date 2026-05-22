// Pico SDK libraries
#include "pico/stdlib.h"
// C common libraries
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
// Custom library headers
#include "vectors.h"

double distance(coords_t coord1, coords_t coord2){
    double lat1 = DEG2RAD((double)coord1.latt_e7 / 1e7), lat2 = DEG2RAD((double)coord2.latt_e7 / 1e7), long1 = DEG2RAD((double)coord1.long_e7 / 1e7), long2 = DEG2RAD((double)coord2.long_e7 / 1e7);
    double dot = sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(long2 - long1);
    if(dot > 1){dot = 1;}
    if(dot < -1){dot = -1;}

    return EARTH_RADIUS_M * acos(dot);
}

double dot(vect3D_t vec1, vect3D_t vec2){
    return ((vec1.x * vec2.x) + (vec1.y * vec2.y) + (vec1.z * vec2.z));
}

double project(vect3D_t vec1, vect3D_t vec2){
    return (dot(vec1, vec2) / dot(vec2, vec2));
}

double vect_len(vect3D_t vect){
    double len = sqrt(dot(vect, vect));
    return len;
}

double vect_angle(vect3D_t vect1, vect3D_t vect2){
    double angle = acos((dot(vect1, vect2))/(vect_len(vect1) * vect_len(vect2)));
    return angle;
}

vect3D_t spherical_to_euclid(coords_t coords){
    double latt = DEG2RAD((double)coords.latt_e7 / 1e7), lon = DEG2RAD((double)coords.long_e7 / 1e7); 
    vect3D_t vect = {EARTH_RADIUS_M * cos(latt) * cos(lon), EARTH_RADIUS_M * cos(latt) * sin(lon), EARTH_RADIUS_M * sin(latt)};
    return vect;
}

vect3D_t vect_subtract(vect3D_t vect1, vect3D_t vect2){
    vect3D_t vect = {(vect1.x - vect2.x), (vect1.y - vect2.y), (vect1.z - vect2.z)};
    return vect;
}

vect3D_t vect_add(vect3D_t vect1, vect3D_t vect2){
    vect3D_t vect = {(vect1.x + vect2.x), (vect1.y + vect2.y), (vect1.z + vect2.z)};
    return vect;
}

void vect_multiply(vect3D_t * vect, double c){
    vect->x = vect->x * c;
    vect->y = vect->y * c;
    vect->z = vect->z * c;
}

void vect_normalize(vect3D_t * vect){
    vect3D_t vect_temp = {vect->x, vect->y, vect->z};
    double size = vect_len(vect_temp);
    vect->x = vect->x / size;
    vect->y = vect->y / size;
    vect->z = vect->z / size;
}

coords_t euclid_to_spherical(vect3D_t vect){
    double lon = RAD2DEG(atan2(vect.y, vect.x)), latt = 90 - RAD2DEG(acos(vect.z / EARTH_RADIUS_M));
    coords_t coords = {(int32_t)(latt * 1e7), (int32_t)(lon * 1e7)};
    return coords;
}

vect3D_t cross_prod(vect3D_t vect1, vect3D_t vect2){
    vect3D_t vect = {(vect1.y * vect2.z - vect1.z * vect2.y), -(vect1.x * vect2.z - vect1.z * vect2.x), (vect1.x * vect2.y - vect1.y * vect2.x)};
    return vect;
}