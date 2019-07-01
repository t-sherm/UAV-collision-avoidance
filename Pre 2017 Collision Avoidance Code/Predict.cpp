#include "Predict.h"


//return the next ten positions
vector<AircraftInfo> predictPath(const AircraftInfo & airplane) {
	/*Time it takes to update position*/ /*I AM UNSURE OF WHAT THIS WILL END UP BEING*/
	double time = 1;

	/*Needed to find the x and y vector velocity. */
	Position calcLongVel, calcLatVel;
	calcLongVel.Latitude = airplane.lastPosition.Latitude;
	calcLongVel.Longitude = airplane.veryLastPosition.Longitude;

	calcLatVel.Latitude = airplane.veryLastPosition.Latitude;
	calcLatVel.Longitude = airplane.lastPosition.Longitude;

	//calculate past velocity. Veloctiy between positions: last and veryLast
	double xVel1 = (gpsDistance(airplane.lastPosition, calcLongVel)) / time;
	double yVel1 = (gpsDistance(airplane.lastPosition, calcLatVel)) / time;

	calcLongVel.Latitude = airplane.currentPosition.Latitude;
	calcLongVel.Longitude = airplane.lastPosition.Longitude;

	calcLatVel.Latitude = airplane.lastPosition.Latitude;
	calcLatVel.Longitude = airplane.currentPosition.Longitude;

	//current velocity. Velocity between postions: current and last
	double xVel2 = (gpsDistance(airplane.currentPosition, calcLongVel)) / time;
	double yVel2 = (gpsDistance(airplane.currentPosition, calcLatVel)) / time;

	if (airplane.lastPosition.Longitude - airplane.veryLastPosition.Longitude < 0) {
		xVel1 = -xVel1;
	}
	if (airplane.lastPosition.Latitude - airplane.veryLastPosition.Latitude < 0) {
		yVel1 = -yVel1;
	}

	if (airplane.currentPosition.Longitude - airplane.lastPosition.Longitude < 0) {
		xVel2 = -xVel2;
	}
	if (airplane.currentPosition.latitude - airplane.lastPosition.latitude < 0) {
		yVel2 = -yVel2;
	}

	//calculate the acceleration
	// a = (v0 - v1) / t; Acceleration equation
	double xAcc2 = (xVel2 - xVel1) / time;
	double yAcc2 = (yVel2 - yVel1) / time;

	//Distance between the current postion and the future position
	//d = v*t + (.5)a*t^2; displacement equation
	double deltaX = xVel2 *time + 0.5*(xAcc2)* pow(time, 2);
	double deltaY = yVel2 *time + 0.5*yAcc2  *pow(time, 2);

	//Holds the 10 aircraft opjects that will have predictive positions and headings
	vector<AircraftInfo> future(10);

	//The first aircraft object has the same positions and heading as the current airplane.
	//This way we can use it in the for loop
	future[0].currentPosition = airplane.currentPosition;
	future[0].lastPosition = airplane.lastPosition;
	future[0].veryLastPosition = airplane.veryLastPosition;
	future[0].Heading = airplane.Heading;

	const float NORTH = 0;
	const float EAST = 90;
	const float SOUTH = 180;
	const float WEST = 270;

	for (int t = 1; t < future.size(); t++) {
		//Using the kinematic equation [displacement = v*t + (.5)a*t^2] and then adding the current position, 
		//we can predict the future position for t timesteps. 
		future[t].currentPosition = meterDisplacement(xVel2 *t + 0.5*(xAcc2)* pow(t, 2),
			yVel2 *t + 0.5*(yAcc2)* pow(t, 2), airplane.currentPosition);

		/*Rest of conditionals is for predicting heading.
		MAY NOT WORK RECOMMEND POSSIBLE REWRITE*/
		if (xVel2 + (xAcc2 *t) == 0) {
			if (yAcc2 > 0) {
				future[t].Heading = EAST;
			}
			else {
				future[t].Heading = WEST;
			}
		}
		else if (yVel2 + (yAcc2 *t) == 0) {
			if (xAcc2 > 0) {
				future[t].Heading = NORTH;
			}
			else {
				future[t].Heading = SOUTH;
			}
		}
		else {
			float h = gpsHeading(future[t].currentPosition, future[t - 1].currentPosition);
			future[t].Heading = h;

		}
	}
	return future;

}

//The first aircraft object in airplanes must be our airplane
Position senseCollision(const AircraftInfo & ourAirplane, const AircraftInfo & otherAirplane) {
	/*Null position has 0 values*/
	Position null;

	//If our airplane has priority, then we do not try to avoid
	if (ourAirplane.priority == 1) return null;

	vector<AircraftInfo> predictionOur;
	vector<AircraftInfo> predictionTheir;

	//get the predicted path for all planes
	predictionOur = predictPath(ourAirplane);
	predictionTheir = predictPath(otherAirplane);



	for (int i = 0; i < predictionTheir.size(); i++) {

		//Create the safety bubble around the predictive positions.
		//The radius grows with each prediction because of the error in predicting positions
		SafetyBubble bubble;
		bubble.position = predictionTheir.at(i).currentPosition;
		// 3 meters + .5 meters after each timestep
		bubble.radius = i*.5 + 10; /*!!!!THE RADIUS OF THE BUBBLE IS STILL UNKNOWN!!!*/

		double distance = gpsDistance(predictionOur[i].currentPosition, predictionTheir[i].currentPosition);

		//If the predictive distance between the airplanes is less than the radius of the bubble
		if (distance < bubble.radius) {
			return avoidCollision(predictionTheir.at(i), ourAirplane, bubble, i);
		}
	}

	return null;

}
