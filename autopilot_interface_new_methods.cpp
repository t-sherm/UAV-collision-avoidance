/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *           Tristan Sherman, <tristan.m.sherman@gmail.com>
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
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 * @author Tristan Sherman, <tristan.m.sherman@gmail.com>
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"

#include <iostream>
#include <string>
#include <fstream>
#include <chrono>
#include <cstring>

//ADDED FOR NEW CA_AVOID CODE
//also added these two lines to autopilot_interface.h
#include <cmath>
#include <tgmath.h>
#define TO_DEGREES 180 / PI
// END -- MAY BE SWITCHED TO OTHER FILE LATER


//ADDED	TO TEST CALLSIGN READING
#include <typeinfo>
using std::endl;
using std::cout;
//END CALLSIGN ADDS


using std::string;
using namespace std::chrono;

//overloaded simple string conversion methods
string convertToString(bool var)
{
	if (var)
		return "TRUE";
	else
		return "FALSE";
}
string convertToString(uint64_t var)
{
	return std::to_string(var);
}

string convertToString(double var)
{
	return std::to_string(var);
}
string convertToString(float var)
{
	return std::to_string(var);
}

string convertToString(int var)
{
	return std::to_string(var);
}
//returns system timestamp as string
string timeStamp()
{

	auto tp = std::chrono::high_resolution_clock::now();
	auto ttime_t = std::chrono::system_clock::to_time_t(tp);
	auto tp_sec = std::chrono::system_clock::from_time_t(ttime_t);
	milliseconds ms = duration_cast<milliseconds>(tp - tp_sec);

	std::tm * ttm = localtime(&ttime_t);

	char date_time_format[] = "%Y.%m.%d-%H.%M.%S";

	char time_str[] = "yyyy.mm.dd.HH-MM.SS.fff";

	strftime(time_str, strlen(time_str), date_time_format, ttm);

	string result(time_str);
	result.append(".");
	result.append(std::to_string(ms.count()));

	return result;
}

//Appends to txt file or creates txt file if none exist
static void addToFile(string line, string description)
{
	std::ofstream file;
	//modify string filepath based on folder you want to access
	//currently will output txt file to project cmake-debug folder
	string filepath = "logging_file.txt";
	file.open(filepath, std::ios::out | std::ios::app);
	//make sure write fails with exception if something is wrong
	file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);
	file << timeStamp() + " " + description + " : " + line << std::endl;
	file.close();
}

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}



/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}







// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	write_count = 0;
	error_counter = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	reading_waypoint_status = 0;
	control_status = 0;      // whether the autopilot is in offboard control mode
	CA_status = 0;	 // whether the collision avoidance thread is running
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id
	CA_tid = 0;//Collision avoidance thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{

	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		//Read message from serial
		mavlink_message_t message;
		success = serial_port->read_message(message);

		//Handle received message
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
				{
					//printf("MAVLINK_MSG_ID_RC_CHANNELS_RAW\n");
					mavlink_msg_rc_channels_raw_decode(&message, &(current_messages.rc_channels_raw_t));
					current_messages.time_stamps.rc_channels_raw_t = get_time_usec();
					this_timestamps.rc_channels_raw_t = current_messages.time_stamps.rc_channels_raw_t;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

				case MAVLINK_MSG_ID_MISSION_COUNT:
				{
					//printf("MAVLINK_MSG_ID_MISSION_COUNT\n");
					mavlink_msg_mission_count_decode(&message, &(current_messages.mavlink_mission_count));
					current_messages.time_stamps.mavlink_mission_count = get_time_usec();
					this_timestamps.mavlink_mission_count = current_messages.time_stamps.mavlink_mission_count;
					break;
				}

				case MAVLINK_MSG_ID_MISSION_ITEM:
				{
					//printf("MAVLINK_MSG_ID_MISSION_ITEM\n");
					mavlink_msg_mission_item_decode(&message, &(current_messages.mavlink_mission_item));
					current_messages.time_stamps.mavlink_mission_item = get_time_usec();
					this_timestamps.mavlink_mission_item = current_messages.time_stamps.mavlink_mission_item;
					break;
				}

				case MAVLINK_MSG_ID_MISSION_CURRENT:
				{
					//printf("MAVLINK_MSG_ID_MISSION_CURRENT\n");
					mavlink_msg_mission_current_decode(&message, &(current_messages.mission_current_t));
					current_messages.time_stamps.mission_current_t = get_time_usec();
					this_timestamps.mission_current_t = current_messages.time_stamps.mission_current_t;

					currentWaypoint = current_messages.mission_current_t.seq;
					
					break;
				}

				case MAVLINK_MSG_ID_ADSB_VEHICLE:
				{
					//printf("\nMAVLINK_MSG_ID_ADS_B\n");
					//Test that this function will work

				
					// TEST CODE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
					//Get Callsign
					//string callsign = mavlink_msg_adsb_vehicle_get_callsign(&message, (current_messages.adsb_vehicle_t.callsign));
					//cout << typeid(mavlink_msg_adsb_vehicle_get_callsign(&message, (current_messages.adsb_vehicle_t.callsign))).name() << endl;
					//cout << mavlink_msg_adsb_vehicle_get_callsign(&message, (current_messages.adsb_vehicle_t.callsign)) << endl;
					//printf("Callsign: %c\n", callsign);


					//Get ICAO address
					uint32_t ICAO = mavlink_msg_adsb_vehicle_get_ICAO_address(&message);
					printf("ICAO %u\n", ICAO);

					if (ICAO == 10700118 ) { //ADSB on test board
					//if (ICAO == 3819648  ) { //ADSB on Piper Cub
											
						// Decode full message and store it
						mavlink_msg_adsb_vehicle_decode(&message, &(current_messages.adsb_vehicle_t));
						current_messages.time_stamps.adsb_vehicle_t = get_time_usec();
						this_timestamps.adsb_vehicle_t = current_messages.time_stamps.adsb_vehicle_t;

						printf("ICAO after decode: %u \n", current_messages.adsb_vehicle_t.ICAO_address);
						//printf("Time since last message: %u \n", current_messages.adsb_vehicle_t.tslc);

					}

					break;
				}



				default:
				{
					//printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all = received_all = recieved_all_messages(this_timestamps);

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}


/*
 * Check if all desired messages have been recieved
 */
bool
Autopilot_Interface::recieved_all_messages(const Time_Stamps &time_stamps) {

    //Returns false only if you want to read something and haven't
    // If you want to read attitude (1) and haven't (0). 1 == 0 ==> false
    // If you don't want do read attitude(0) and haven't(0). 0 == 0 ==> true
    return
        (messages_to_read.read_attitude == time_stamps.attitude)
        //&&  (messages_to_read.read_autopilot_version == time_stamps.autopilot_version)
        &&  (messages_to_read.read_battery_status == time_stamps.battery_status)
        &&  (messages_to_read.read_global_position_int == time_stamps.global_position_int)
        &&  (messages_to_read.read_heartbeat == time_stamps.heartbeat)
        &&  (messages_to_read.read_highres_imu == time_stamps.highres_imu)
        &&  (messages_to_read.read_local_position_ned == time_stamps.local_position_ned)
        &&  (messages_to_read.read_position_target_global_int == time_stamps.position_target_global_int)
        &&  (messages_to_read.read_position_target_local_ned == time_stamps.position_target_local_ned)
        &&  (messages_to_read.read_radio_status == time_stamps.radio_status)
        &&  (messages_to_read.read_sys_status == time_stamps.sys_status);
        //&&  (messages_to_read.read_power_status == time_stamps.power_status);
}




// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	//uses pthread helper function &start_mavlink_interface_read_thread
   	//to call function start_read_thread in a new thread
	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		
		if ( time_to_exit )
		return;
		printf("Reboot Autopilot\n");

		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}



	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;



}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);
	pthread_join(CA_tid,NULL);

	// now the predict, read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		//printf("Read message\n");
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}





// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	current_setpoint = sp;

	// write a message and signal writing
	write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		write_setpoint();
	}

	// signal end
	writing_status = false;

	return;

}



/*
 * Change PWM value of servo. (Moves Servo)
 */
void
Autopilot_Interface::
write_set_servo(const int &servo, const int &pwm)
{
    printf("Changing PWM value of servo %d to %d\n", servo, pwm);
    writing_status = true;
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------


    mavlink_command_long_t cmd;
    cmd.target_system = system_id;
    cmd.target_component = autopilot_id;
    cmd.command = MAV_CMD_DO_SET_SERVO;
    cmd.param1 = servo;
    cmd.param2 = pwm;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id,companion_id, &message, &cmd);


    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);

    // check the write
    if ( len <= 0 )
        fprintf(stderr,"WARNING: could not send MAV_CMD_DO_SET_SERVO \n");

    writing_status = false;
}


// End Autopilot_Interface

/*
 * Send waypoints to pixhawk
 */
void
Autopilot_Interface::
write_waypoints(std::vector<mavlink_mission_item_t> waypoints) {

    printf("Sending Waypoints\n");
    writing_status = true;

    //Pixhawk needs to know how many waypoints it will receive
    mavlink_mission_count_t mission_count;
    mission_count.count = (int) waypoints.size();
    if(send_waypoint_count(mission_count) <= 0){
        fprintf(stderr,"WARNING: could not send waypoint count \n");
    }

    //Send all waypoints
    for(int i = 0; i < waypoints.size(); i++){
        waypoints[i].target_system = system_id;
        waypoints[i].target_component = autopilot_id;
        waypoints[i].frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        waypoints[i].autocontinue = true;
        waypoints[i].current = 1;
        mavlink_message_t message;
        mavlink_msg_mission_item_encode(system_id, companion_id, &message, &waypoints[i]);

        printf("Waypoint %d Lat: %f, Long: %f, Alt: %f, Seq: %d\n", i, waypoints[i].x, waypoints[i].y, waypoints[i].z, waypoints[i].seq);
        if ( write_message(message) <= 0 )
            fprintf(stderr,"WARNING: could not send MAV_CMD_DO_SET_SERVO \n");
    }

    writing_status = false;
}

/*
 * Tells pixhawk how many waypoints it will recieve
 */
int
Autopilot_Interface::
send_waypoint_count(mavlink_mission_count_t mavlink_mission_count) {
    mavlink_mission_count.target_system = system_id;
    mavlink_mission_count.target_component = autopilot_id;
    mavlink_message_t message;
    mavlink_msg_mission_count_encode(system_id, autopilot_id, &message, &mavlink_mission_count);
    return write_message(message);
}






// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------


void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}

void*
start_collision_avoidance_thread (void *args)
{

	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->collision_avoidance_begin();

	// done!
	return NULL;

}
/*
 * Returns a mavlink acceptable waypoint
 */
mavlink_mission_item_t
Autopilot_Interface::create_waypoint(const float &lat, const float &lon, const int &alt, const int &wp_number, const int &radius) {
    mavlink_mission_item_t mission_item;
    mission_item.command = MAV_CMD_NAV_WAYPOINT;
    mission_item.param1 = 0;//hold time in decimal second IGNORED by ArduPlane
    mission_item.param2 = radius;//Acceptance radius in meters
    mission_item.param3 = 0;//0 to pass through WP if >0 radius in meters to pass by WP
    mission_item.param4 = NAN;//Desired yaw angle NaN for unchanged
    mission_item.x = lat;//latitude
    mission_item.y = lon;//longitude
    mission_item.z = alt;//altitude
    mission_item.seq = wp_number;//waypoint number
    return mission_item;
}

/*Return a new position given the x and y displacement in meters of a waypoint
  The waypoint is created off an existing mission item*/
mavlink_mission_item_t
Autopilot_Interface::
createNewDisplacedWaypoint(const double & deltaX, const double & deltaY, const mavlink_mission_item_t & b){

    //coordinate offset in Radians
    float deltaLat = (deltaY / RADIUS_EARTH);
    float deltaLong = deltaX / (RADIUS_EARTH * cos(b.x * PI / 180));

    mavlink_mission_item_t newPosition = b;
    newPosition.x = b.x + (deltaLat * (180 / PI));
    newPosition.y = b.y + deltaLong * ((180 / PI));
    //newPosition.seq = b.seq +1;    
    return newPosition;

}




















//Return a new position given the x and y displacement in meters of an avoid waypoint
//The waypoint is created off of the current position of the aircraft
mavlink_mission_item_t
Autopilot_Interface::
NewAvoidWaypoint(const double & deltaX, const double & deltaY, aircraftInfo & pos){
 
    //INPUTS: deltaX and deltaY in meters
    //	      pos: current lat,lon position of the aircraft
    // NOTE: This function uses NED frame. Therefore, +x = North = dLat
    //                                                +y = East  = dLon    


    //Convert from cartesian to NED frame. For ease of understanding
    double deltaX_NED = deltaY;
    double deltaY_NED = deltaX;

    //coordinate offset in Radians
    double deltaLat =  deltaX_NED / RADIUS_EARTH; //This may need to be updated later
    double deltaLon = deltaY_NED / RADIUS_EARTH;

    mavlink_mission_item_t newPosition;
    
    newPosition.x = pos.lat[0] + (deltaLat * (180 / PI));
    newPosition.y = pos.lon[0] + (deltaLon * (180 / PI));

    //newPosition.seq = seq;    
    return newPosition;

}



//Return x and y distance (NED frame) from current position to second position
mavlink_mission_item_t
Autopilot_Interface::
distanceVectors(const double &target_lat, const double &target_long, const double &current_lat, const double &current_long){
 
    //INPUTS: Current position and second position
    // NOTE: This function uses NED frame. Therefore, +x = North = dLat
    //                                                +y = East  = dLon    


	 //Find change in lattitude and longitude in radians
    double deltaLat = (target_lat - current_lat) * (PI/180);
	 double deltaLon = (target_long - current_long) * (PI/180);

	 //Find distance in meters
	 double distX = deltaLat * RADIUS_EARTH;
    double distY = deltaLon * RADIUS_EARTH;

    mavlink_mission_item_t newPosition;

	 //This must be changed later. For now we use mavlink_mission_item_t to get the job done. What we are really outputting is x and y distance
    newPosition.x = distX; //Distance N/S (meters)
    newPosition.y = distY; // Distance E/W (meters)

    return newPosition;

}



















/*
	Calculates the distance between the target's latitude and longitude coordinates and the plane's latitude and longitude coordinates
	Calculates in meters
*/
double
Autopilot_Interface::
gpsDistance(const double &target_lat, const double &target_long, const double &current_lat, const double &current_long)
{

    double tLatRad = target_lat * TO_RADIANS;
    double pLatRad = current_lat * TO_RADIANS;
    double deltaLat = (current_lat - target_lat) * TO_RADIANS;
    double deltaLong = (current_long - target_long) * TO_RADIANS;

    double var = sin(deltaLat / 2) * sin(deltaLat / 2) + (cos(tLatRad) * cos(pLatRad)) * (sin(deltaLong / 2) * sin(deltaLong / 2));
    double c = 2 * atan2(sqrt(var), sqrt(1 - var));

    return RADIUS_EARTH * c;

}


/*
double
Autopilot_Interface::
predictDistance( const float &ourPos, const predictedCollision &otherPos)
{
	double aLatRad = ourAircraft.lat[2]*TO_RADIANS;
	double bLatRad = otherAircraft.lat[2]*TO_RADIANS;
	double deltaLat = (otherAircraft.lat[2] - ourAircraft.lat[2])* TO_RADIANS;
	double deltaLong = (otherAircraft.lon[2] - otherAircraft.lon[2])* TO_RADIANS;

	double var = sin(deltaLat / 2) * sin(deltaLat / 2) + (cos(aLatRad) * cos(bLatRad)) * (sin(deltaLong / 2) * sin(deltaLong / 2));
	double c = 2 * atan2(sqrt(var), sqrt(1 - var));
	double distance = c * RADIUS_EARTH;

	
	double bubbleRadius = t*.5 + 5;
	// nested if to check the two collision points to check if they are close enough to tell if there is a collision 

	double rH = abs(ourAircraft.Hdg - otherAircraft.Hdg);
	cout << "Relative Heading " << t << ": " << rH << "\n" << "Distance " << t << ": " << distance << "\n\n";
		
}


*/


















//This function will tell the pixhawk to travel to this waypoint next
//This will override the previous next waypoint
void
Autopilot_Interface::
setCurrentWaypoint( uint16_t &waypointNum )
{

	mavlink_mission_set_current_t newWaypoint;

	//Set parameters for the message
	newWaypoint.seq = waypointNum;
	newWaypoint.target_system = system_id;
	newWaypoint.target_component = companion_id;


	//writing_status = true;

	mavlink_message_t message;
	mavlink_msg_mission_set_current_encode(system_id, companion_id, &message, &newWaypoint);
	//writing_status = false;	


        if ( write_message(message) <= 0 )
	fprintf(stderr,"WARNING: could not send MAV_CMD_DO_SET_SERVO \n");

	//printf("The pixhawk is now traveling to waypoint %u\n", newWaypoint.seq);
}


//mavlink_mission_item_int_t
void
Autopilot_Interface::
Request_Waypoints()
{

//This function requests a mission according to this guide: http://qgroundcontrol.org/mavlink/waypoint_protocol

	if (reading_waypoint_status != 0) {
		printf("ERROR: WAYPOINTS ALREADY REQUESTED\n");
		return;
	}

	printf("REQUESTING WAYPOINTS\n");
	reading_waypoint_status = true;

	//Request mission
	//mavlink_mission_request_list_t missionRequest;
	mavlink_message_t message; 

	mavlink_msg_mission_request_list_pack(system_id, companion_id, &message, system_id, companion_id, 0);
	if ( write_message(message) <= 0 )
	fprintf(stderr,"WARNING: could not send MAV_CMD_REQUEST_WAYPOINTS \n");

	//For some reason this function needs to end if the waypoint message is to be read
	Receive_Waypoints();
	
	//printf("Current mission size: %lu\n", currentMission.size());
	reading_waypoint_status = false;

}



void
Autopilot_Interface::
Receive_Waypoints() {

	//printf("RECEIVING WAYPOINTS\n"); 

	//Receive waypoint count
	int counter = 0;
	mavlink_mission_count_t missionCount = current_messages.mavlink_mission_count;
	
	bool waypointError = true;
	while ( waypointError ) {

		//Give the autopilot some time to receive the message
		while (missionCount.count < 0.1 && counter < 1000) {
	
			missionCount = current_messages.mavlink_mission_count;

			counter++;
			usleep(10000);
		}
	
		waypointError = false;

		if (missionCount.count < 0.1 && !time_to_exit) {
		printf("TIMEOUT: MISSION COUNT NOT RECEIVED\n");
		waypointError = true;
		reading_waypoint_status = false;
		Request_Waypoints();
		break;
		}

		
		if (missionCount.count > 1000)  {
			int i;
			for (i=0; i<100; i++){

				usleep(100000);
			}

			

			if (missionCount.count > 1000) {

				waypointError = true;
				printf("WAYPOINT COUNT NOT RECEIVED: TRY AGAIN\n");
				reading_waypoint_status = false;
				error_counter++;
				Request_Waypoints();
				
				return;
			}
			printf("ERROR: WAYPOINT COUNT NOT RECEIVED\n");
			return;
		}

	}

	//---------------------------------------------------------------------
	//Send request then receive each waypoint of the mission
	//---------------------------------------------------------------------

	int seqNum = 0; // The current waypoint we are requesting and receiving
 	mavlink_message_t message;
	mavlink_mission_item_t missionItem = current_messages.mavlink_mission_item;


	printf("Number of waypoints: %i\n\n", missionCount.count);

	for (seqNum = 0;  seqNum < missionCount.count;  seqNum++) {

 
		//---------------------------------------------------------------------------------------
		//Request waypoint
		//---------------------------------------------------------------------------------------

		mavlink_msg_mission_request_pack(system_id, companion_id, &message, system_id, companion_id, seqNum, 0);
		if ( write_message(message) <= 0 )
			fprintf(stderr,"WARNING: could not send MAV_CMD_REQUEST_WAYPOINTS \n");
			usleep(1700);//Wait for response


		//--------------------------------------------------------------------------------------
		//Receive waypoint
		//--------------------------------------------------------------------------------------

		missionItem = current_messages.mavlink_mission_item;

		//Account for whether the mission already has elements in it
		if (currentMission.size() > seqNum) { currentMission[seqNum] = missionItem; }
		else { currentMission.push_back(missionItem); }


		//Makes sure the waypoint is written to the right vector element and isn't skipped


		int i = 0;		
		//printf("seqNum: %i\n", seqNum);
		//printf("current mission seq: %i\n", currentMission[seqNum].seq);
		
		while (seqNum != currentMission[seqNum].seq && !waypointError && !time_to_exit) {

			missionItem = current_messages.mavlink_mission_item;
			currentMission[seqNum] = missionItem;
			usleep(1000);		
			i++;
			/* Accounts for a frozen code. Needs improvement
			if (i=100000) { //If the autopilot does not receive the waypoint in this amount of time, an error has occured so start over.

				printf("hey %i\n", i);
				reading_waypoint_status = false;
				Request_Waypoints();
				break;
				
			}*/

		}
		

	}
	

}



//-------------------------------------------------------------------
//Collision avoidance starter functions
//-------------------------------------------------------------------


aircraftInfo::
aircraftInfo() {

	//Initializes all values to zero
	//Descriptions are in the aircraftInfo struct
	lat [3] = {0};
	lon [3] = {0};
	alt [3] = {0};
	velocityX [2] = {0};
	velocityY [2] = {0};
	xAcc = 0;
	yAcc = 0;
	futureDistx [3] = {0};
	futureDisty [3] = {0};
	Hdg = 0;
	safetyBubble = 5;  //Roughly 60 ft
	priority = 0;

}

void
Autopilot_Interface::
start_collision_avoidance()
{

	int result;
	

	//Start thread
	result = pthread_create( &CA_tid, NULL, &start_collision_avoidance_thread, this );
	if ( result ) throw result;

	printf("START COLLISION AVOIDANCE THREAD\n\n");
}

void
Autopilot_Interface::
collision_avoidance_begin()
{

	if ( CA_status != 0 )
	{
		fprintf(stderr,"Collision Avoidance thread already running\n");
		return;
	}
	else
	{
		CA_predict_thread();
		return;
	}

}


void
Autopilot_Interface::
insert_waypoint ( mavlink_mission_item_t &newWaypoint, uint16_t &desiredSeqNumber ) {

		//-----------------------------------------------------------------------
		//Insert waypoint
		//-----------------------------------------------------------------------


		//printf("Current mission size: %lu\n", currentMission.size());

			currentMission.push_back(currentMission[0]); //Takes first element and copies it to last for dummy data
			int endVal = currentMission.size() - desiredSeqNumber;
			int i;

			for (i = 0; i < endVal; i++) {
												//printf("\nLatitude before %f\n", currentMission.rbegin()[i].x);
				currentMission.rbegin()[i] = currentMission.rbegin()[(i+1)]; //Shifts waypoints over until the inserted waypoint is reached
				currentMission.rbegin()[i].seq = currentMission.size() - (i+1);
												//printf("Latitude after %f\n", currentMission.rbegin()[i].x);

			}
	
		newWaypoint.z = currentMission[endVal].z; //I chose endVal because it is guaranteed to always be there
		newWaypoint.seq = desiredSeqNumber;
		currentMission[desiredSeqNumber] = create_waypoint(newWaypoint.x, newWaypoint.y, currentMission[endVal].z, desiredSeqNumber, 15);


	//Write avoid waypoint within mission
	write_waypoints(currentMission);

}


void
Autopilot_Interface::
CA_predict_thread()
{

	CA_status = true;

	predictedCollision collision;
	mavlink_global_position_int_t gpos = current_messages.global_position_int;
	mavlink_adsb_vehicle_t adsb = current_messages.adsb_vehicle_t;
	mavlink_mission_item_t otherVelocity; //FOR TESTING PURPOSES. MUST BE CHANGED LATER ALONG WITH "distanceVectors" FUNCTION
	

	int AVOID_DELAY = 0;

	//printf("ho\n %f\n %f\n %f\n", otherAircraft.lat[0] / 1E7, otherAircraft.lat[1] / 1E7, otherAircraft.lat[2] / 1E7);
/*
	while (ourAircraft.lat[2] < 0.001 && ourAircraft.lat[2] > -0.001 && ! time_to_exit) {
	

		//update stored messages
		gpos = current_messages.global_position_int;
		adsb = current_messages.adsb_vehicle_t;

		//Update our aircraft position
		ourAircraft.lat[2] = ourAircraft.lat[1];
		ourAircraft.lon[2] = ourAircraft.lon[1];

		ourAircraft.lat[1] = ourAircraft.lat[0];		
		ourAircraft.lon[1] = ourAircraft.lon[0];

		ourAircraft.lat[0] = gpos.lat;
		ourAircraft.lon[0] = gpos.lon;
			

		//Update other aircraft position
		otherAircraft.lat[2] = otherAircraft.lat[1];
		ourAircraft.lon[2] = otherAircraft.lon[1];

		otherAircraft.lat[1] = otherAircraft.lat[0];		
		ourAircraft.lon[1] = otherAircraft.lon[0];

		otherAircraft.lat[0] = adsb.lat;
		otherAircraft.lon[0] = adsb.lon;
			
		//Update our aircraft velocity
		ourAircraft.velocityX[1] = ourAircraft.velocityX[0];
		ourAircraft.velocityY[1] = ourAircraft.velocityY[1];		

		ourAircraft.velocityX[0] = gpos.vx;
		ourAircraft.velocityY[0] = gpos.vy;

		//Update other aircraft velocity
		otherAircraft.velocityX[1] = otherAircraft.velocityX[0];
		otherAircraft.velocityY[1] = otherAircraft.velocityY[1];		

		otherAircraft.velocityX[0] = adsb.hor_velocity;
		otherAircraft.velocityY[0] = adsb.ver_velocity;

		printf("\nWAITING FOR AIRCRAFT POSITION\n");
		printf("CURRENT STATUS: %f %f %f\n", ourAircraft.lat[0] / 1E7, ourAircraft.lat[1] / 1E7, ourAircraft.lat[2] / 1E7);
		sleep(1);
		}
*/

	printf("\nPOSITION LOGGED\n");
	//------------------------------------------------
	//Start predict loop
	//------------------------------------------------
	double vx;
	double vy;
	//Prediction will occur once both aircraft vectors are updated
	bool ourUpdated = false;
	bool otherUpdated = false;
	int fractionSinceUpdate = 0;
	while ( ! time_to_exit ) {


		//update stored messages
		gpos = current_messages.global_position_int;
		adsb = current_messages.adsb_vehicle_t;


		///---------------------------------
      ///
      /// Gather All Parameters for Our Plane, such as lat,lon,velocity,acceleration, Heading,
      ///
      ///----------------------------------

		
		
		//double test = ourAircraft.lat[0];

		//printf("our lattitude: %lf\n", (double) gpos.lat/1E7);

		//printf("stored lattitude: %lf\n", ourAircraft.lat[0]);
		//double test2 = (double) gpos.lat/1E7;

		//double output =  fabs(test - test2)*100000.00;

		//printf("difference: %lf\n", output );


		//If the current stored position is not equal to the gpos position i.e. the position message has been updated, then these vectors can be 			updated. Otherwise wait a little longer to update the array
		if ( fabs(ourAircraft.lat[0] - (double) gpos.lat/1E7) > 0.000000000001 || fabs(ourAircraft.lon[0] - (double) gpos.lon/1E7) > 0.00000000001 ) {
			



			//Update our aircraft position (X, Y) A
			ourAircraft.lat[2] = ourAircraft.lat[1];
			ourAircraft.lon[2] = ourAircraft.lon[1];

			ourAircraft.lat[1] = ourAircraft.lat[0];		
			ourAircraft.lon[1] = ourAircraft.lon[0];

			ourAircraft.lat[0] = gpos.lat / 1E7;
			ourAircraft.lon[0] = gpos.lon / 1E7;
			
			//Update our aircraft velocity (v) A, and heading
			ourAircraft.velocityX[1] = ourAircraft.velocityX[0];
			ourAircraft.velocityY[1] = ourAircraft.velocityY[0];		

			ourAircraft.velocityX[0] = gpos.vy / 100.0;//NOTE: EXPERIMENTING WITH SWITCHING DIRECTIONS. Log output indicated that the velocity should be (y,x) rather than (x,y)
			ourAircraft.velocityY[0] = gpos.vx / 100.0;

			ourAircraft.Hdg = gpos.hdg / 100.0;
			
			//Update our aircraft acceleration (acc) A
			ourAircraft.xAcc = (ourAircraft.velocityX[0] - ourAircraft.velocityX[1]);//xAcc
			ourAircraft.yAcc = (ourAircraft.velocityY[0] - ourAircraft.velocityY[1]);//yAcc
			
			//Now that we have updated the position, lets inform the other functions
			ourUpdated = true;
		
		}


		//If the position has not been updated in the current messages then start counting
		if ( fabs(otherAircraft.lat[0] - (double) adsb.lat/1E7) < 0.00000000001 || fabs(otherAircraft.lon[0] - (double) adsb.lon/1E7) < 0.0000000001)  {
			fractionSinceUpdate++; //Time = .3333*fractionSinceUpdate 
		}
		

		//printf("log criteria dist: %lf\n",(double) abs(otherAircraft.lat[0] - (double) adsb.lat/1E7));
		if ( fractionSinceUpdate > 20 ) { //Get rid of zeros in otherAircraft vector[2]
			//Update other aircraft position (X, Y) B
			//Update other aircraft position (X, Y) B
			otherAircraft.lat[2] = otherAircraft.lat[1];
			otherAircraft.lon[2] = otherAircraft.lon[1];

			otherAircraft.lat[1] = otherAircraft.lat[0];		
			otherAircraft.lon[1] = otherAircraft.lon[0];

			otherAircraft.lat[0] = adsb.lat / 1E7;
			otherAircraft.lon[0] = adsb.lon / 1E7;

			//Update other aircraft velocity (v) B
			otherAircraft.velocityX[1] = otherAircraft.velocityX[0];
			otherAircraft.velocityY[1] = otherAircraft.velocityY[0];	
			
			//Assume that we are receiving information at once a second. That is not a correct assumption but I need to get this shizzle tested somehow
			otherVelocity = distanceVectors(otherAircraft.lat[0], otherAircraft.lon[0], otherAircraft.lat[1], otherAircraft.lon[1]); // divide by time between time stamps

			//Current velocity (INPUT TO PREDICT IS CARTESIAN. THEREFORE WE CONVERT TO CARTESIAN HERE)
			otherAircraft.velocityY[0] = otherVelocity.x;
			otherAircraft.velocityX[0] = otherVelocity.y;

			//Update other aircraft acceleration (acc) B
			otherAircraft.xAcc = (otherAircraft.velocityX[0] - otherAircraft.velocityX[1]);//xAcc
			otherAircraft.yAcc = (otherAircraft.velocityY[0] - otherAircraft.velocityY[1]);//yAcc
			
		}



		if ( fabs(otherAircraft.lat[0] - (double) adsb.lat/1E7) > 0.00000000001 || fabs(otherAircraft.lon[0] - (double) adsb.lon/1E7) > 0.0000000001 ){
			printf("logged!");
			//Update other aircraft position (X, Y) B
			otherAircraft.lat[2] = otherAircraft.lat[1];
			otherAircraft.lon[2] = otherAircraft.lon[1];

			otherAircraft.lat[1] = otherAircraft.lat[0];		
			otherAircraft.lon[1] = otherAircraft.lon[0];

			otherAircraft.lat[0] = adsb.lat / 1E7;
			otherAircraft.lon[0] = adsb.lon / 1E7;

			//Update other aircraft velocity (v) B
			otherAircraft.velocityX[1] = otherAircraft.velocityX[0];
			otherAircraft.velocityY[1] = otherAircraft.velocityY[0];	
			
			//Assume that we are receiving information at once a second. That is not a correct assumption but I need to get this shizzle tested somehow
			otherVelocity = distanceVectors(otherAircraft.lat[0], otherAircraft.lon[0], otherAircraft.lat[1], otherAircraft.lon[1]); // divide by time between time stamps

			//Current velocity (INPUT TO PREDICT IS CARTESIAN. THEREFORE WE CONVERT TO CARTESIAN HERE)
			otherAircraft.velocityY[0] = otherVelocity.x;
			otherAircraft.velocityX[0] = otherVelocity.y;

			//Update other aircraft acceleration (acc) B
			otherAircraft.xAcc = (otherAircraft.velocityX[0] - otherAircraft.velocityX[1]);//xAcc
			otherAircraft.yAcc = (otherAircraft.velocityY[0] - otherAircraft.velocityY[1]);//yAcc
			
			
			//Now that we have updated the position, lets inform the other functions
			otherUpdated = true;
			fractionSinceUpdate = 0;

		}

		//printf("Time since update: %i\n",fractionSinceUpdate);
		printf("Ours updated? %d\nOther updated? %d\n", ourUpdated, otherUpdated);

		printf("First condition satisfied: %d\n Second condition satisfied: %d\n",(ourUpdated && otherUpdated), (fractionSinceUpdate > 20 ));

		//Predict now happens if both the ownships and the other aircraft's position has changed. Or it has been more than 3 seconds
		if ((ourUpdated == true && otherUpdated == true) || (fractionSinceUpdate > 20 )) { /*9 is 3 seconds since update speed is 1/3 of a second*/
				
			printf("PREDICT\n");
			//printf("1/3 seconds since ADS-B update: %i", fractionSinceUpdate);
		
			//log all the new position data
			addToFile("New point ~~~~~~~~~~","");
			addToFile(convertToString(ourAircraft.lat[2]),"last lattitude");
			addToFile(convertToString(ourAircraft.lon[2]),"last ongitude");
			addToFile(convertToString(ourAircraft.lat[1]),"second to last lattitude");
			addToFile(convertToString(ourAircraft.lon[1]),"second to last Longitude");
			addToFile(convertToString(ourAircraft.lat[0]),"Our lattitude");
			addToFile(convertToString(ourAircraft.lon[0]),"Our Longitude");
			addToFile(convertToString(otherAircraft.lat[1]),"Other second Lattitude");
			addToFile(convertToString(otherAircraft.lon[1]),"Other second Longitude");
			addToFile(convertToString(otherAircraft.lat[0]),"Other Lattitude");
			addToFile(convertToString(otherAircraft.lon[0]),"Other Longitude");
			addToFile(convertToString(ourAircraft.velocityX[0]),"Our X Velocity");//Cartesian (E/W)
			addToFile(convertToString(ourAircraft.velocityY[0]),"Our Y Velocity");//Cartesian (N/S)
			addToFile(convertToString(otherAircraft.velocityX[0]),"Other X Velocity");//Cartesian
			addToFile(convertToString(otherAircraft.velocityY[0]),"Other Y Velocity");//Cartesian
		
			//Predict using the logged point
			collision = CA_Predict(ourAircraft, otherAircraft);
			//collision.collisionDetected == true;
			
			//printf("Collision predicted? %d\n", collision.collisionDetected);
		
			//Avoid if necessary
			if (collision.collisionDetected == true && AVOID_DELAY <=1 ) {

				AVOID_DELAY = 20; //This is not a great way to keep multiple points from being written to the pixhawk 
				CA_Avoid(ourAircraft, otherAircraft, collision);
			}


			//printf("Distance between aircraft: %f\n", gpsDistance(ourAircraft.lat[0], ourAircraft.lon[0], otherAircraft.lat[0], otherAircraft.lon[0]));

			if (AVOID_DELAY > 0) {
				AVOID_DELAY--;
				printf("Waiting for aircraft to avoid...\n");
				//addToFile("", "Waiting to avoid...");
			}
			
			//Now that we have predicted we can revert the update boolians
			ourUpdated = false;
			otherUpdated = false;
			//printf("end predict with other updated = %d\n",otherUpdated);
			
		}//End predict



		
		//Wait a third of a second before updating the next position
		usleep(333333);
		//usleep(1000000);
		

	}

	CA_status = false;

}


predictedCollision
Autopilot_Interface::
CA_Predict(aircraftInfo & aircraftA, aircraftInfo & aircraftB) {

	int fps = 10; //fps meaning future points
	double  rH; // for relative Heading of the planes
	int t;
	float t2;
	//-----------------------------------------------------------------------------
	//Predict t timesteps into the future
	//-----------------------------------------------------------------------------

	for (t = 0; t < fps; t++)
	{

	t2 = t + .01;
		//----------------------------------------------------------------------------------------------------------
		//
		//	This predicts our future position, .001 timesteps in front of that position, 
		//	    and the distance between those two points
		//--------------------------------------------------------------------------------------------------------


		aircraftA.futureDistx[0] = aircraftA.velocityX[0] * t + 0.5*(aircraftA.xAcc)* pow(t, 2);//futureDistx
		aircraftA.futureDisty[0] = aircraftA.velocityY[0] * t + 0.5*(aircraftA.yAcc)* pow(t, 2);//futureDisty

		aircraftA.futureDistx[1] = aircraftA.velocityX[1] * (t2) + 0.5*(aircraftA.xAcc)* pow((t2 + 0.001), 2);//futureDistx2
		aircraftA.futureDisty[1] = aircraftA.velocityY[1] * (t2) + 0.5*(aircraftA.yAcc)* pow((t2 + 0.001), 2);//futureDisty2

		aircraftA.futureDistx[2] = aircraftA.futureDistx[1] - aircraftA.futureDistx[0];//dx
		aircraftA.futureDisty[2] = aircraftA.futureDisty[1] - aircraftA.futureDisty[0];//dy


		//---------------------------------------------------------------------------------------------------------
		//	The Other Aircraft's prediction for future positions
		//---------------------------------------------------------------------------------------------------------


		aircraftB.futureDistx[0] = aircraftB.velocityX[0] * t + 0.5*(aircraftB.xAcc)* pow(t, 2);//futuredistx
		aircraftB.futureDisty[0] = aircraftB.velocityY[0] * t + 0.5*(aircraftB.yAcc)* pow(t, 2);//futuredisty

		aircraftB.futureDistx[1] = aircraftB.velocityX[1] * (t2) + 0.5*(aircraftB.xAcc)* pow((t + 0.1), 2);//futuredistx2
		aircraftB.futureDisty[1] = aircraftB.velocityY[1] * (t2) + 0.5*(aircraftB.yAcc)* pow((t + 0.1), 2);//futuredisty2

		aircraftB.futureDistx[2] = aircraftB.futureDistx[1] - aircraftB.futureDistx[0];//dx to find tangent
		aircraftB.futureDisty[2] = aircraftB.futureDisty[1] - aircraftB.futureDisty[0];//dy to find tangent

		//printf("Future distance A (%f, %f, %f)\n", aircraftA.futureDistx[0], aircraftA.futureDistx[1], aircraftA.futureDistx[2]);

		//------------------------------------------------------------------------------------------------------
		//
		//	This is the possible cases for our planes heading
		//
		//-----------------------------------------------------------------------------------------------------

		/*/Accounts for the rare case that the aircraft is exactly North, South, East, or West
		if (aircraftA.velocityX[0] == 0 && aircraftA.velocityX[1] == 0 && aircraftA.velocityY[0] > 0 && aircraftA.velocityY[1] > 0) {
			aircraftA.Hdg = 0;
		}

		else if (aircraftA.velocityY[0] == 0 && aircraftA.velocityY[1] == 0 && aircraftA.velocityX[0] > 0 && aircraftA.velocityX[1] > 0) {
			aircraftA.Hdg = 90;
		}


		else if (aircraftA.velocityX[0] == 0 && aircraftA.velocityX[1] && aircraftA.velocityY[0] < 0 && aircraftA.velocityY[1] < 0) {
			aircraftA.Hdg = 180;
		}

		else if (aircraftA.velocityY[0] == 0 && aircraftA.velocityY[1] == 0 && aircraftA.velocityX[0] < 0 && aircraftA.velocityX[1] < 0) {
			aircraftA.Hdg = 270;
		}

		//Calculates heading at any angle
		else if (aircraftA.futureDistx[0] > 0 && aircraftA.futureDisty[0] > 0) {
			aircraftA.Hdg = 90 - (atan(((abs(aircraftA.futureDisty[2])) / (abs(aircraftA.futureDistx[2]))))*(180 / PI));
		}


		else if (aircraftA.futureDistx[0] > 0 && aircraftA.futureDisty[0] < 0) {
			aircraftA.Hdg = 180 - (atan(((abs(aircraftA.futureDisty[2])) / (abs(aircraftA.futureDistx[2]))))*(180 / PI));
		}

		else if (aircraftA.futureDistx[0] < 0 && aircraftA.futureDisty[0] < 0) {
			aircraftA.Hdg = 360 - (atan(((abs(aircraftA.futureDisty[2])) / (abs(aircraftA.futureDistx[2]))))*(180 / PI));
		}

		else if (aircraftA.futureDistx[0] < 0 && aircraftA.futureDisty[0] > 0) {
			aircraftA.Hdg = 270 - (atan(((abs(aircraftA.futureDisty[2])) / (abs(aircraftA.futureDistx[2]))))*(180 / PI));
		}

		printf("Plane 1 Heading %f: %f\n", t, aircraftA.Hdg);



		//----------------------------------------------------------------------------------------
		//
		// Calculates the other planes possible headings
		//
		//----------------------------------------------------------------------------------------
		
		//This may be redundant. This can be converted into a function
		if (aircraftB.velocityX[0] == 0 && aircraftB.velocityX[1] == 0 && aircraftB.velocityY[0] > 0 && aircraftB.velocityY[1] > 0) {
			aircraftB.Hdg = 0;
		}

		else if (aircraftB.velocityX[0] == 0 && aircraftB.velocityX[1] == 0 && aircraftB.velocityY[0] < 0 && aircraftB.velocityY[1] < 0) {
			aircraftB.Hdg = 180;
		}

		else if (aircraftB.velocityY[0] == 0 && aircraftB.velocityY[1] == 0 && aircraftB.velocityX < 0 && aircraftB.velocityX < 0) {
			aircraftB.Hdg = 270;
		}

		else if (aircraftB.velocityY[0] == 0 && aircraftB.velocityY[1] == 0 && aircraftB.velocityX[0] > 0 && aircraftB.velocityX[1] > 0) {
			aircraftB.Hdg = 90;
		}


		//Calculates heading at any angle
		if (aircraftB.futureDistx[0] > 0 && aircraftB.futureDisty[0] > 0) {
			aircraftB.Hdg = 90 - (atan(((abs(aircraftB.futureDisty[2])) / (abs(aircraftB.futureDistx[2]))))*(180 / PI));
		}

		else if (aircraftB.futureDistx[0] > 0 && aircraftB.futureDisty[0] < 0) {
			aircraftB.Hdg = 180 - (atan(((abs(aircraftB.futureDisty[2])) / (abs(aircraftB.futureDistx[2]))))*(180 / PI));
		}

		else if (aircraftB.futureDistx[0] < 0 && aircraftB.futureDisty[0] < 0) {
			aircraftB.Hdg = 360 - (atan(((abs(aircraftB.futureDisty[2])) / (abs(aircraftB.futureDistx[2]))))*(180 / PI));
		}

		else if (aircraftB.futureDistx[0] < 0 && aircraftB.futureDisty[0] > 0) {
			aircraftB.Hdg = 270 - (atan(((abs(aircraftB.futureDisty[2])) / (abs(aircraftB.futureDistx[2]))))*(180 / PI));
		}

		printf("Plane 2 Heading %f: %f\n", t, aircraftB.Hdg);



		//--------------------------------------------------------------------------------------------------
		//Relative heading
		//--------------------------------------------------------------------------------------------------

		rH = abs(aircraftB.Hdg - aircraftA.Hdg);
		*/
		//-----------------------------------------------------------------------------------------------
		//
		//	This is converting our current values into gps (not finished yet) [here we use the Lat/Lon array to store the planes position in gps]
		//
		//-----------------------------------------------------------------------------------------------


		//Creates a future position item based on the current position and future distance
		mavlink_mission_item_t ourFuturePos = NewAvoidWaypoint(aircraftA.futureDistx[0], aircraftA.futureDisty[0], aircraftA);
		//printf("Predicted position: (%f, %f)\n", ourFuturePos.x, ourFuturePos.y);

		//Log
		addToFile(convertToString(t), "Time interval");
		addToFile(convertToString(ourFuturePos.x), "ourFuturePos.x");
		addToFile(convertToString(ourFuturePos.y), "ourFuturePos.y");

		mavlink_mission_item_t otherFuturePos = NewAvoidWaypoint(aircraftB.futureDistx[0], aircraftB.futureDisty[0], aircraftB);
		
		//Log
		addToFile(convertToString(otherFuturePos.x), "otherFuturePos.x");
		addToFile(convertToString(otherFuturePos.y), "otherFuturePos.y");



		//----------------------------------------------------------------------------------------------
		//
		//	Compares the future positions of each plane to see if they are within the safety bubble
		//
		//----------------------------------------------------------------------------------------------

		int bubbleRadius = aircraftA.safetyBubble;
		double predictedDistance = gpsDistance(ourFuturePos.x, ourFuturePos.y, otherFuturePos.x, otherFuturePos.y);

		//Log
		addToFile(convertToString(predictedDistance), "Future predicted distance");
		//printf("Predicted distance between planes: %f m\n", predictedDistance);
		collisionPoint.collisionDetected = false;

		//	printf("NEED TO CHANGE: Collision detected if predicted distance <=1\n");
		if (predictedDistance <= bubbleRadius) { //MODIFIED FOR LOGGING PURPOSES!!! NOT OFFICIAL CODE

			collisionPoint.collisionDetected = true;
			collisionPoint.timeToCollision = t;
			collisionPoint.relativeHeading = rH;
			
			//printf("Collision detected\n");
			addToFile("COLLISION DETECTED", "");
			printf("Predicted distance between planes at collision: %f m\n", predictedDistance);
			//addToFile(convertToString(predictedDistance), "Distance at predicted collision");
			return collisionPoint;
		}
	}

	printf("\n\n");
	return collisionPoint;
}





void 
Autopilot_Interface::
CA_Avoid( aircraftInfo & aircraftA, aircraftInfo & aircraftB, predictedCollision &collision)
{

//Stores waypoints in Current_Waypoints
    Request_Waypoints();

    //get info for aircraft A
    double latA = aircraftA.lat[0];
    double lonA = aircraftA.lon[0];
    uint64_t buffA = aircraftA.safetyBubble;
    //get info for aircraft B
    double latB = aircraftB.lat[0];
    double lonB = aircraftB.lon[0];
    uint64_t buffB = aircraftB.safetyBubble;

    //This will allow the distance of the new waypoint to be smaller or larger depending on the timeToCollision
    AVOID_BUFFER = avoidBufferSize + (buffB / collision.timeToCollision);
    if (collision.timeToCollision < 1)
        AVOID_BUFFER = avoidBufferSize;
    //print and log statements
    //printf("avoid buffer size: %i\n", avoidBufferSize);
    addToFile(convertToString(AVOID_BUFFER), "avoidBufferSize");

    // Length k = distance needed between avoidWaypoint and B
    float k = aircraftB.safetyBubble + AVOID_BUFFER;
    addToFile(convertToString(k), "k: distance between waypoint and B");
    //angular distance
    float Ad = k/RADIUS_EARTH;
	//Log
	addToFile(convertToString(Ad), "Ad: Angular distance");


    //Bearing from A to B: from https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
    double dLat = latB - latA;
    double dLon = lonB - lonA;
    double x = TO_DEGREES * (cos(latB * TO_RADIANS) * sin(dLon * TO_RADIANS));
    double y = TO_DEGREES * (cos(latA * TO_RADIANS) * sin(latB * TO_RADIANS) - sin(latA * TO_RADIANS) * cos(latB * TO_RADIANS) * cos(dLon * TO_RADIANS));

	//Log
	addToFile(convertToString(dLat), "dLat");
	addToFile(convertToString(dLon), "dLon");
	addToFile(convertToString(x), "x");
	addToFile(convertToString(y), "y");

    //Special cases in dealing with NED frame
    double bearingAB = TO_DEGREES * atan2(x * TO_RADIANS,y * TO_RADIANS);   //Try changing (y,x) --> atan2(x,y) to switch to NED frame

	addToFile(convertToString(bearingAB), "bearingAB");


	/* This helped in MATLAB but may be wrong here
    if ((dLon < 0) && (dLat > 0) || (dLon > 0) && (dLat < 0)) {
        bearingAB += 180;
	}
	*/

    //Slope of line BC: right now it is simply a 75 degree turn
    double bearingBC = bearingAB + 75;
	//Log
	addToFile(convertToString(bearingBC), "bearingBC");

    //displacement in meters of new waypoint in x direction (North/South)
    double lat = TO_DEGREES*( asin( sin(latA*TO_RADIANS) * cos(Ad*TO_RADIANS) + cos(latA*TO_RADIANS)* sin(Ad*TO_RADIANS) * cos(bearingBC*TO_RADIANS)));
	
    //Longitude displacement
    double lon_disp = TO_DEGREES * (  atan2( sin(bearingBC*TO_RADIANS)*sin(Ad*TO_RADIANS)*cos(latA*TO_RADIANS), cos(Ad*TO_RADIANS)-sin(latA*TO_RADIANS)*sin(latB*TO_RADIANS)));
    double lon = lonA + lon_disp;


    //generate the waypoint
    mavlink_mission_item_t avoidWaypoint;
    avoidWaypoint.x = lat;
    avoidWaypoint.y = lon;

    //log its lat and lon
    addToFile(convertToString(avoidWaypoint.x), "avoidWP lat (x)");
    addToFile(convertToString(avoidWaypoint.y), "avoidWP lon (y)");

	//See if distance is correct
	addToFile(convertToString(gpsDistance(avoidWaypoint.x, avoidWaypoint.y, aircraftA.lat[0], aircraftA.lon[0])), "Distance to avoid point");

    //insert the waypoint
    insert_waypoint( avoidWaypoint, currentWaypoint);


    //Tell the aircraft to go to the waypoint
    setCurrentWaypoint(currentWaypoint);
	addToFile(convertToString(currentWaypoint), "Replaced waypoint and current waypoint");
    printf("Collision point created\n");

}
