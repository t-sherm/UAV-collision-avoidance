#pragma once
#include "Message.h"
#include "AircraftInfo.h"
#include "Predict.h"
class CollisionAvoidance {
public:
	//Initialize CollisionAvoidance
	CollisionAvoidance(const uint8_t & ourID, const uint8_t & otherID);

	//update an Aircraft's info and position. Calls other updateFunctions
	void update(const Message & message);

	//True if collision will occur, false otherwise
	bool avoid();

	//If collision then use this to get avoid waypoint
	Position getAvoidWaypoint();

private:

	//Aircraft objects
	AircraftInfo ourAircraft;
	AircraftInfo otherAircraft;
	/*Recommend using vector for numerous airplanes: aircrafts<>*/

	/*Best to not use this constructor*/
	CollisionAvoidance();

	//Grab this using the getter when a collision is going to happen
	Position avoidWaypoint;

	void setAvoidWaypoint(const Position& waypoint);

	void updatePosition(AircraftInfo & aircraft, const Message & message);
	void updateAircraftInfo(AircraftInfo & aircraft, const Message & message);
};