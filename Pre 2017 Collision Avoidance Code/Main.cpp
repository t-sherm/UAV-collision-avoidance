
#include <iostream>
#include <vector>
//#include "Position.h"
//consider replacing Position.h with VehicleInertialState.h
//#include "VehicleInertialState.h"
#include "AircraftInfo.h"
#include "GPSandConstants.h"
#include "CollisionAvoidance.h"
using namespace std;

//Testing program test
int main() {
	CollisionAvoidance ourCA(2, 3);
	CollisionAvoidance otherCA(3, 2);


	Message ourMessage1, ourMessage2, ourMessage3;
	ourMessage1.Vehicle_ID = 2;
	ourMessage1.Latitude = 33.9337335;
	ourMessage1.Longitude = -117.6302129;
	ourMessage1.Altitude = 0;
	ourMessage1.Heading = 180;
	ourMessage1.Velocity = 0;
	ourMessage1.priority = 0;
	ourMessage1.timestamp = 0;

	ourMessage2.ID = 2;
	ourMessage2.Latitude = 33.9337335;
	ourMessage2.Longitude = -117.6303041;
	ourMessage2.Altitude = 0;
	ourMessage2.Heading = 180;
	ourMessage2.Velocity = 0;
	ourMessage2.priority = 0;
	ourMessage2.timestamp = 0;

	ourMessage3.Vehicle_ID = 2;
	ourMessage3.Latitude = 33.9337335;
	ourMessage3.Longitude = -117.6303846;
	ourMessage3.Altitude = 0;
	ourMessage3.Heading = 180;
	ourMessage3.Velocity = 0;
	ourMessage3.priority = 0;
	ourMessage3.timestamp = 0;

	Message otherMessage1, otherMessage2, otherMessage3;
	otherMessage1.Vehicle_ID = 3;
	otherMessage1.Latitude = 33.9337513;
	otherMessage1.Longitude = -117.6319885;
	otherMessage1.Altitude = 0;
	otherMessage1.Heading = 0;
	otherMessage1.Velocity = 0;
	otherMessage1.priority = 1;
	otherMessage1.timestamp = 0;

	otherMessage2.ID = 3;
	otherMessage2.Latitude = 33.9337557;
	otherMessage2.Longitude = -117.6319207;
	otherMessage2.Altitude = 0;
	otherMessage2.Heading = 0;
	otherMessage2.Velocity = 0;
	otherMessage2.priority = 1;
	otherMessage2.timestamp = 0;

	otherMessage3.ID = 3;
	otherMessage3.Latitude = 33.9337557;
	otherMessage3.Longitude = -117.6318169;
	otherMessage3.Altitude = 0;
	otherMessage3.Heading = 0;
	otherMessage3.Velocity = 0;
	otherMessage3.priority = 1;
	otherMessage3.timestamp = 0;

	ourCA.update(ourMessage1);
	ourCA.update(ourMessage2);
	ourCA.update(ourMessage3);
	ourCA.update(otherMessage1);
	ourCA.update(otherMessage2);
	ourCA.update(otherMessage3);

	otherCA.update(otherMessage1);
	otherCA.update(otherMessage2);
	otherCA.update(otherMessage3);
	otherCA.update(ourMessage1);
	otherCA.update(ourMessage2);
	otherCA.update(ourMessage3);

	if (ourCA.avoid()) {
		cout << "Lat: " << ourCA.getAvoidWaypoint().Latitude << endl;
		cout << "Long: " << ourCA.getAvoidWaypoint().Longitude << endl;
	}

	if (otherCA.avoid()) {
		cout << "Lat: " << otherCA.getAvoidWaypoint().Latitude << endl;
		cout << "Long: " << otherCA.getAvoidWaypoint().Longitude << endl;
	}
	return 0;
}

