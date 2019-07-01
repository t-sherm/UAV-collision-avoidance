#pragma once
#include <stdint.h>
struct Position {
	//Initialized to 0 to see if it has been initialized before. I'm using 0 as null value.
	Position() : Longitude(0), Latitude(0), Altitude(0), timestamp(0) {};
	float64_t Latitude;
	float64_t Longitude;
	float64_t Altitude;

	uint8_t timestamp;
};
