#pragma once

// Custom Libraries
#include "vectors.h"
#include "hardware.h"
// MAVLink libraries
#include <mavlink/common/mavlink.h>
#include <mavlink/minimal/mavlink.h>
#include <mavlink/standard/mavlink.h>
// C common libraries
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

bool check_geofence(geofence_t*, mavlink_global_position_int_t);
bool check_return(coords_t, mavlink_global_position_int_t);
coords_t calculate_return(geofence_t*, int32_t, int32_t);
void geofence_setup(coords_t*, geofence_t*, uint8_t);