#include "CollisionAvoidance.h"

CollisionAvoidance::CollisionAvoidance(const uint8_t & ourID, const uint8_t & otherID) {
	ourAircraft.Vehicle_ID = ourID;
	otherAircraft.Vehicle_ID = otherID;
	//Recommend using vectors when implementing numerous airplanes
	//i.e aircrafts<i>.id = id<i>
}

CollisionAvoidance::CollisionAvoidance() {
	ourAircraft.Vehicle_ID = 0;
	otherAircraft.Vehicle_ID = 1;
	/*I HAVE NO IDEA WHAT THE DEFAULT ID's SHOULD BE
	best to never use this constructor*/
}


void CollisionAvoidance::update(const Message & message) { //updates an aircraft's information given a message
	if (message.ID == ourAircraft.Vehicle_ID) {
		updateAircraftInfo(ourAircraft, message);
	}
	else {
		updateAircraftInfo(otherAircraft, message);
	}
	/*For numerous airplanes recommend using ID as index
	i.e updateAircraftInfo(aircrafts<message.ID>, message)
	this will not work if ID's aren't {0,1,..,n} */

}

bool CollisionAvoidance::avoid() {
	/*Reset the avoidWaypoint (THIS COULD BE DANGEROUS if the avoidWaypoint wasn't grabbed before
	recommend using a flag and a seperate reset function)*/
	avoidWaypoint.Altitude = 0;
	avoidWaypoint.Latitude = 0;
	avoidWaypoint.Longitude = 0;
	avoidWaypoint.timestamp = 0;

	//If we have at least 3 previous positions we can try to predict a collision
	if (ourAircraft.veryLastPosition.Longitude != 0 && otherAircraft.veryLastPosition.Longitude != 0) {
		setAvoidWaypoint(senseCollision(ourAircraft, otherAircraft));
	}
	//If an avoidWaypoint was created then there is a potential collision
	if (avoidWaypoint.Longitude != 0) {
		return true;
	}
	else {
		return false;
	}
}

void CollisionAvoidance::setAvoidWaypoint(const Position & waypoint) {
	avoidWaypoint = waypoint;
}

/*Updates positions since AircraftInfo hold the last three known positions*/
void CollisionAvoidance::updatePosition(AircraftInfo & aircraft, const Message & message) {
	aircraft.veryLastPosition.Longitude = aircraft.lastPosition.Longitude;
	aircraft.veryLastPosition.Latitude = aircraft.lastPosition.Latitude;
	aircraft.veryLastPosition.Altitude = aircraft.lastPosition.Altitude;
	aircraft.veryLastPosition.timestamp = aircraft.lastPosition.timestamp;

	aircraft.lastPosition.Longitude = aircraft.currentPosition.Longitude;
	aircraft.lastPosition.Latitude = aircraft.currentPosition.Latitude;
	aircraft.lastPosition.Altitude = aircraft.currentPosition.Altitude;
	aircraft.lastPosition.timestamp = aircraft.currentPosition.timestamp;

	aircraft.currentPosition.Longitude = message.Longitude;
	aircraft.currentPosition.Latitude = message.Latitude;
	aircraft.currentPosition.Altitude = message.Altitude;
	aircraft.currentPosition.timestamp = message.timestamp;
}

void CollisionAvoidance::updateAircraftInfo(AircraftInfo & aircraft, const Message & message) {
	updatePosition(aircraft, message);
	aircraft.priority = message.priority;
	aircraft.velocity = message.Velocity;
	aircraft.heading = message.Heading;
}

Position CollisionAvoidance::getAvoidWaypoint() {
	return avoidWaypoint;
}
