#pragma once
#include "AircraftInfo.h"
#include "SafetyBubble.h"
#include "Position.h"
#include "Avoid.h"
#include "GPSandConstants.h"
#include <math.h>
#include <vector>
#include <iostream>
using namespace std;

vector<AircraftInfo> predictPath(const AircraftInfo & airplane);

Position senseCollision(const AircraftInfo & ourAirplane, const AircraftInfo & otherAirplane);