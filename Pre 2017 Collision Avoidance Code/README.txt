How To use Collision Avoidance

The CollisionAvoidance class is the interface. 
Any data or behavior needed from collision avoidance is done through this class. 

1. CollisionAvoidance constructor requires the ID of both planes to work properly

	signature:	CollisionAvoidance(const uint8_t & ourID, const uint8_t & otherID);
		

2. update function must be called each time the gps location of the airplane is updated 
		
	signature:	void update(const Message & message)// Note: See what Message consists of
		
3. to check for collision: call avoid function

	signature:	bool avoid(); // True if collision will occur 
	
	3.1 If a collision will occur then grab newWaypoint using the getter. This is the 
		waypoint the plane must take in order to avoid the collision. 
		
		signature: Position getNewWaypoint(); //Note: See what Position consists of
		
		
/*I am assuming we will recieve something that has everything in this struct*/
struct Message {
	float latitude;
	float longitude;
	float altitude;

	float velocity;
	float heading;

	uint8_t ID;
	uint8_t timestamp;
	uint8_t priority;
};


struct Position {
	//Initialized to 0, I'm using 0 as null value.
	Position() : longitude(0), latitude(0), altitude(0), timestamp(0) {};
	
	float latitude;
	float longitude;
	float altitude;

	uint8_t timestamp; //Unecessary for newWaypoint
};
	

