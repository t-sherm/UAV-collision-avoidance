/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *	     Tristan Sherman, <tristan.m.sherman@gmail.com>
 *	     Mitchell Caudle, Jimmy Lopez, Hana Haideri
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.h
 *
 * @brief Autopilot interface definition
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"

#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <vector>

#include "Mavlink/common/mavlink.h"

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

                                                // bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111



#define PI 3.14159265359
#define TO_RADIANS PI / 180;
#define RADIUS_EARTH	6378037.0



// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
uint64_t get_time_usec();
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp);
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);

void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);
void* start_collision_avoidance_thread(void *args); //Runs 2nd after start_collision_avoidance()

// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;
	uint64_t adsb_vehicle_t;
	uint64_t mavlink_mission_count;
	uint64_t mavlink_mission_item;
	uint64_t mission_current_t;
	uint64_t rc_channels_raw_t;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
		battery_status = 0;
		radio_status = 0;
		local_position_ned = 0;
		global_position_int = 0;
		position_target_local_ned = 0;
		position_target_global_int = 0;
		highres_imu = 0;
		attitude = 0;
	}

};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Radio Status
	mavlink_radio_status_t radio_status;

	//Raw input channels
	mavlink_rc_channels_raw_t rc_channels_raw_t;

	// Local Position
	mavlink_local_position_ned_t local_position_ned;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;

	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;

	// HiRes IMU
	mavlink_highres_imu_t highres_imu;

	// Attitude
	mavlink_attitude_t attitude;

    	//version
    	mavlink_autopilot_version_t autopilot_version;


    	//power status
    	mavlink_power_status_t power_status_t;

    	// ack of commands
    	mavlink_command_ack_t mavlink_command_ack;

    	// ack of missions cmd
    	mavlink_mission_ack_t mavlink_mission_ack;

    	//mission item return from mission request by sending seq number
    	mavlink_mission_item_t mavlink_mission_item;

    	//mission current seq number
    	mavlink_mission_current_t mavlink_mission_current;

    	//mission count returned from mission request list only seq numbers
    	mavlink_mission_count_t mavlink_mission_count;

    	//mission item reached
    	mavlink_mission_item_reached_t malink_mission_item_reached;

	//current waypoint that the pixhawk is traveling to
	mavlink_mission_current_t mission_current_t;


	//--------------------------------------------------------------
	// Additional System Parameters
	//--------------------------------------------------------------

	//ADS-B Information
	mavlink_adsb_vehicle_t adsb_vehicle_t;
	




	// Time Stamps
	Time_Stamps time_stamps;

	void
	reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};


/*
 * Boolean value for the desired messages to be read
 * Ex: To read battery status. Set read_battery_status to true
 */
struct Read_Messages {


    bool read_heartbeat;
    bool read_sys_status;
    bool read_battery_status;
    bool read_radio_status;
    bool read_local_position_ned;
    bool read_global_position_int;
    bool read_position_target_local_ned;
    bool read_position_target_global_int;
    bool read_highres_imu;
    bool read_attitude;
    bool read_autopilot_version;
    bool read_power_status;
    bool read_mission_ack;
    bool read_command_ack;

    /*Default to not read any messages*/
    Read_Messages()
    {
        read_heartbeat = false;
        read_sys_status = false;
        read_battery_status = false;
        read_radio_status = false;
        read_local_position_ned = false;
        read_global_position_int = false;
        read_position_target_local_ned = false;
        read_position_target_global_int = false;
        read_highres_imu = false;
        read_attitude = false;
        read_autopilot_version = false;
        read_power_status = false;
        read_mission_ack = false;
        read_command_ack = false;

    }

};


	//gh

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * Autopilot Interface Class
 *
 * This starts two threads for read and write over MAVlink. The read thread
 * listens for any MAVlink message and pushes it to the current_messages
 * attribute.  The write thread at the moment only streams a position target
 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
 * is changed by using the method update_setpoint().  Sending these messages
 * are only half the requirement to get response from the autopilot, a signal
 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
 * method.  Signal the exit of this mode with disable_offboard_control().  It's
 * important that one way or another this program signals offboard mode exit,
 * otherwise the vehicle will go into failsafe.
 */



// ------------------------------------------------------------------------------
//   Helper Struct
// ------------------------------------------------------------------------------
struct Waypoint{
    double lat;
    double lon;
    int alt;
    Waypoint(double lat = 0, double lon = 0, int alt = 0){
        this->lat = lat;
        this->lon = lon;
        this->alt = alt;
    }
};


struct aircraftInfo {


	aircraftInfo();

	//Aircraft positions
	// 0: current position, 1: last position (one second in the past), 2: very last position (two seconds in the past)
	double lat [3];
	double lon [3];
	double alt [3];

	//Aircraft velocity and accelerations
	//0: current velocity/acceleration, 1: last velocity (one second in the past)
	float velocityX [2];
	float velocityY [2];
	double xAcc;
	double yAcc;

	//Information for the predict function
	//0: Predicted point
	//1: Point that is .01 second in front of the predicted point
	//2: Distance between predicted point 0 and 1
	double futureDistx [3];
	double futureDisty [3];

	//Predicted heading
	float Hdg;
	
/*	Creats a safety bubble around the aircraft that increases in radius with each
	predicted future way point up to 10 way points	*/
	uint64_t safetyBubble; 

	uint8_t priority;

};

struct predictedCollision {

	float timeToCollision;
	bool collisionDetected;
	float relativeHeading;
};

class Autopilot_Interface
{

public:

	Autopilot_Interface();
	Autopilot_Interface(Serial_Port *serial_port_);
	~Autopilot_Interface();

	char reading_status;
	char writing_status;
	char control_status;
	char CA_status;
	uint64_t write_count;

   	int system_id;
	int autopilot_id;
	int companion_id;

	Mavlink_Messages current_messages;
	mavlink_set_position_target_local_ned_t initial_position;
	Read_Messages messages_to_read; //Messages to read

	void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
	void read_messages();
	int  write_message(mavlink_message_t message);

	void enable_offboard_control();
	void disable_offboard_control();

	void start();
	void stop();

	void start_read_thread();
	void start_write_thread(void);

	void handle_quit( int sig );

	//------------------------------------------------------------------------
	//Collision avoidance definitions
	//------------------------------------------------------------------------
	


	char reading_waypoint_status;
	int error_counter;
	uint16_t currentWaypoint;
	float AVOID_BUFFER;

	// This plays a part in how large the AVOID_BUFFER will be
	const int avoidBufferSize = 100;
	

	


	aircraftInfo ourAircraft, otherAircraft;
	std::vector<mavlink_mission_item_t> currentMission;
	predictedCollision collisionPoint;




	//---------------------------------------------------
	//New functions
	//---------------------------------------------------

	void write_set_servo(const int &servo, const int &pwm);

	void write_waypoints(std::vector<mavlink_mission_item_t> waypoints);

	bool recieved_all_messages(const Time_Stamps &time_stamps);

	int send_waypoint_count(mavlink_mission_count_t mavlink_mission_count);

	mavlink_mission_item_t create_waypoint(const float &lat, const float &lon, const int &alt,const int &wp_number,
                                               const int &radius);

	mavlink_mission_item_t createNewDisplacedWaypoint(const double & deltaX, const double & deltaY, const mavlink_mission_item_t & b);

	double gpsDistance(const double &target_lat, const double &target_long, const double &current_lat, const double &current_long);

	void setCurrentWaypoint( uint16_t &waypointNum );//Tells the autopilot to go to a specific, pre-loaded waypoint

	void Request_Waypoints();
	void Receive_Waypoints();
	void insert_waypoint ( mavlink_mission_item_t &newWaypoint, uint16_t &desiredSeqNumber );


	//Collision Avoidance functions

	void start_collision_avoidance(); //Runs first to start collision avoidance

	void collision_avoidance_begin(); //Runs 3rd to check if the collision avoidance thread is running

	void CA_predict_thread(); //Runs 4th: actual predict thread

	predictedCollision CA_Predict(aircraftInfo & aircraftA, aircraftInfo & aircraftB);
	
	void CA_Avoid( aircraftInfo & aircraftA, aircraftInfo & aircraftB, predictedCollision &collision);

	//Creates an avoid waypoint
	mavlink_mission_item_t NewAvoidWaypoint(const double & deltaX, const double & deltaY, aircraftInfo & pos);

private:

	Serial_Port *serial_port;

	bool time_to_exit;

	pthread_t read_tid;
	pthread_t write_tid;
	pthread_t CA_tid;

	mavlink_set_position_target_local_ned_t current_setpoint;

	void read_thread();
	void write_thread(void);

	int toggle_offboard_control( bool flag );
	void write_setpoint();

};



class Collision_Avoidance
{
	public:
	int randint;
};




#endif // AUTOPILOT_INTERFACE_H_

