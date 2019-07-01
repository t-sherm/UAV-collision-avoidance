#pragma once
#include <math.h>
#include "Position.h"

#define DELTA_T	0.5 /* change in time per timestep */ // Old code unsure if necessary 

#define MAX_TIMESTEP	20.0 /* maximum timestep considered */ // Old code unsure if necessary 

#define GPS_ERROR	10.0 /* gps error in meters */ // Old code unsure if necessary 

#define AVG_VELOCITY	17.88 /* average aircraft velocity in m/s */ // Old code unsure if necessary 

#define PI 3.14159265359

#define TO_RADIANS  PI / 180;

/* in meters */
#define CIRC_OF_EARTH	40075160.0
#define RADIUS_EARTH	6378037.0


double gpsDistance(const Position & a, const Position & b);

Position meterDisplacement(const double & deltaX, const double & deltaY, const Position & b);

float gpsHeading(const Position & previous, const Position & current);

/*Old code unsure if necessary*/
Position gpsOffset(Position start, double dlatitude, double dlongitude);

