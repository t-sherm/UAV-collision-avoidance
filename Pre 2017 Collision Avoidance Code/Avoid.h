#pragma once
#include "Position.h"
#include "AircraftInfo.h"
#include "SafetyBubble.h"
#include "GPSandConstants.h"
#include <cmath> // sqrt needed

Position avoidCollision(const AircraftInfo & otherPlaneInfo, const AircraftInfo & ourPlaneInfo, const SafetyBubble & otherPlaneBubble,
	const int & timeToCollision);