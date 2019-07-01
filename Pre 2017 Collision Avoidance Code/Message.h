#pragma once
#include <stdint.h>
/*I am assuming we will recieve something that has everything in this struct*/
struct Message {
	float Latitude;
	float Longitude;
	float Altitude;

	float Velocity;
	float Heading;

	uint8_t ID;
	uint8_t timestamp;
	uint8_t priority;
};
