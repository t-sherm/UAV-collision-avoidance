#pragma once
#include "Position.h"

struct SafetyBubble {
	/* circle used as buffer zone for possible collisions (i.e. hitbox)*/
	Position position;

	/*Radius in meters*/
	float radius;
};