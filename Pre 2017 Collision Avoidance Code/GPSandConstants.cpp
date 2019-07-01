#include "GPSandConstants.h"

/* distance between 2 gps points in meters*/
double gpsDistance(const Position & a, const Position & b) {

	double aLatRad = a.Latitude*TO_RADIANS;
	double bLatRad = b.Latitude*TO_RADIANS;
	double deltaLat = (b.Latitude - a.latitude) * TO_RADIANS;
	double deltaLong = (b.Longitude - a.longitude)*TO_RADIANS;

	double var = sin(deltaLat / 2) * sin(deltaLat / 2) + (cos(aLatRad) * cos(bLatRad)) * (sin(deltaLong / 2) * sin(deltaLong / 2));
	double c = 2 * atan2(sqrt(var), sqrt(1 - var));

	return RADIUS_EARTH * c;

}

/*Return a new position given the x and y displacement in meters of a Position*/
Position meterDisplacement(const double & deltaX, const double & deltaY, const Position & b) {
	//coordinate offset in Radians
	float deltaLat = (deltaY / RADIUS_EARTH);
	float deltaLong = deltaX / (RADIUS_EARTH * cos(b.Latitude * PI / 180));

	Position newPosition;
	newPosition.Latitude = b.Latitude + (deltaLat * (180 / PI));
	newPosition.Longitude = b.Longitude + deltaLong * ((180 / PI));
	return newPosition;
}

/* find new gps point given a start point and a displacement*/
Position gpsOffset(Position start, double dlatitude, double dlongitude)
{
	Position toReturn;

	toReturn.Latitude = start.Latitude + dlatitude*(360.0 / CIRC_OF_EARTH);
	toReturn.Longitude = start.Longitude + dlongitude*(360.0 / CIRC_OF_EARTH)*(1 / cos(start.Latitude*(PI / 180.0)));

	toReturn.Altitude = start.Altitude;

	return toReturn;
}

/* find approximate heading from previous and current gps points
MAY NOT WORK RECOMMEND REWRITE*/
float gpsHeading(const Position & previous, const Position & current)
{

	float previousLatRad = previous.Latitude * TO_RADIANS;
	float previousLongRad = previous.Longitude * TO_RADIANS;

	float currentLatRad = current.Latitude * TO_RADIANS;
	float currentLongRad = current.Longitude * TO_RADIANS;

	float deltaLong = currentLongRad - previousLongRad;

	float x = sin(deltaLong) * cos(currentLatRad);
	float y = cos(previousLatRad)* sin(currentLatRad) - sin(previousLatRad)*cos(currentLatRad)*cos(deltaLong);
	float heading = atan2(y, x) * (180 / PI) + 180;
	if (heading >= 360) heading -= 360;
	return heading;
}
