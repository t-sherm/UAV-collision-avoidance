#pragma once
#include "Position.h"
#include <stdint.h>
struct AircraftInfo {
	Position currentPosition;
	Position lastPosition;
	Position veryLastPosition;

	float32_t Velocity;
	float32_t Heading;
	uint8_t priority;
	uint16_t Vehicle_ID;

};
