/* Joshua Tellez
* Northrop Grumman Collaboration Project
* Collision Avoidance : Avoid Team
* Avoid2016.cpp : Avoid predicted aerial collision by creating a new waypoint for our plane to follow
*/

#include "Avoid.h" 
/* otherPlaneInfo = predictive information, not current; This plane will not move
* ourPlaneInfo = current information, not predictive; This plane will move out of otherPlane’s way
* otherPlaneBubble = predictive Safety Bubble of otherPlane (plane we’re trying to avoid )
* timeToCollision = How much time does our airplane have until it will crash with the otherPlane
*/
Position avoidCollision(const AircraftInfo & otherPlaneInfo, const AircraftInfo & ourPlaneInfo, const SafetyBubble & otherPlaneBubble,
	const int & timeToCollision) {

	bool turnRight; // True if plane2 has to turn right and false if plane2 has to turn left

					// If heading1 >= 180 
	if (otherPlaneInfo.Heading >= 180) {

		// turnRight if  [ (heading1 - 180) <= heading2 <= heading1 ]
		turnRight = (((otherPlaneInfo.Heading - 180) <= ourPlaneInfo.Heading) && (ourPlaneInfo.Heading <= otherPlaneInfo.Heading));

	}
	// heading1 < 180 
	else {

		// turnLeft if [ heading1 < heading2 < (heading1 + 180) ]
		turnRight = !((otherPlaneInfo.Heading < ourPlaneInfo.Heading) && (ourPlaneInfo.Heading < (otherPlaneInfo.Heading + 180)));
		// Therefore, do not turn right if you have to turn left 
	}

	// This will determine how large the AVOID_BUFFER will be
	const int arbitraryConstant = 10; /*!!!!! THIS IS YET TO BE DETERMINED!!!! */

									  //This will allow the distance of the new waypoint to be smaller or larger depending on the timeToCollision
	const float AVOID_BUFFER = arbitraryConstant / (timeToCollision * otherPlaneBubble.radius);

	// Length k = distance needed between avoidWaypoint and B 
	float k = otherPlaneBubble.radius + AVOID_BUFFER;

	Position avoidWaypoint; // Referred to as C in pseudocode

	Position A = ourPlaneInfo.currentPosition;
	Position B = otherPlaneBubble.position;

	// Guard against division by zero when finding slopes
	/*!!!!!Increment value(.00000001) IS YET TO BE DETERMINED!!!!!*/
	if (B.longitude - A.longitude == 0) B.longitude += .00000001;
	if (B.latitude - A.latitude == 0) B.latitude += .00000001;

	// Slope of line AB
	double slopeAB = (B.latitude - A.latitude) / (B.longitude - A.longitude);

	// Slope of line BC, such that C = avoidWaypoint.
	double slopeBC = -1 / slopeAB;

	// xDisplacement = k * ( 1 / ((1 + slopeBC^2)^(1/2)) )
	double xDisplacement = k * (1 / sqrt(1 + (slopeBC*slopeBC)));

	// yDisplacement = k * ( slopeBC / ((1 + slopeBC^2)^(1/2)) )
	double yDisplacement = k * (slopeBC / sqrt(1 + (slopeBC*slopeBC)));

	// If( Ax > Bx and Ay >= By )
	if (A.Longitude > B.Longitude && A.Latitude >= B.Latitude) {
		if (turnRight) {
			avoidWaypoint = meterDisplacement(-xDisplacement, yDisplacement, B);
		}
		else {
			avoidWaypoint = meterDisplacement(xDisplacement, -yDisplacement, B);
		}
	}
	// else if ( Ax <= Bx and Ay > By )
	else if (A.Longitude <= B.Longitude && A.Latitude > B.Latitude) {
		if (turnRight) {
			avoidWaypoint = meterDisplacement(-xDisplacement, -yDisplacement, B);
		}
		else {
			avoidWaypoint = meterDisplacement(xDisplacement, yDisplacement, B);
		}
	}
	// else if (Ax < Bx and Ay <= By)
	else if (A.Longitude < B.Longitude && A.Latitude <= B.Latitude) {
		if (turnRight) {
			avoidWaypoint = meterDisplacement(xDisplacement, -yDisplacement, B);
		}
		else {
			avoidWaypoint = meterDisplacement(-xDisplacement, yDisplacement, B);
		}
	}
	// else if (Ax >= Bx and Ay < By )
	else {
		if (turnRight) {
			avoidWaypoint = meterDisplacement(xDisplacement, yDisplacement, B);
		}
		else {
			avoidWaypoint = meterDisplacement(-xDisplacement, -yDisplacement, B);
		}
	}

	avoidWaypoint.Altitude = A.Altitude;

	return avoidWaypoint;

}
