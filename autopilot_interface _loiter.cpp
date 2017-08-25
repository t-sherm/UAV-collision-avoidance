/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *           Tristan Sherman, <tristan.m.sherman@gmail.com>
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
					//printf("MAVLINK_MSG_ID_ADS_B\n");
					mavlink_msg_adsb_vehicle_decode(&message, &(current_messages.adsb_vehicle_t));
					current_messages.time_stamps.adsb_vehicle_t = get_time_usec();
					this_timestamps.adsb_vehicle_t = current_messages.time_stamps.adsb_vehicle_t;
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
        waypoints[i].frame = MAV_FRAME_GLOBAL;
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
 
    //coordinate offset in Radians
    float deltaLat = (deltaY / RADIUS_EARTH);
    float deltaLong = deltaX / (RADIUS_EARTH * cos(pos.lon[0] * PI / 180));

    mavlink_mission_item_t newPosition;
    
    newPosition.x = pos.lon[0] + (deltaLong * (180 / PI));
    newPosition.y = pos.lat[0] + deltaLat * ((180 / PI));
    //newPosition.seq = seq;    
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
	safetyBubble = 20;  //Roughly 60 ft
	priority = 0;

}

void
Autopilot_Interface::
start_collision_avoidance()
{

	int result;
	

	//Start predict thread
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
CA_predict_thread()
{

	CA_status = true;

	predictedCollision collision;
	mavlink_global_position_int_t gpos = current_messages.global_position_int;
	mavlink_adsb_vehicle_t adsb = current_messages.adsb_vehicle_t;

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

	while ( ! time_to_exit ) {


		//update stored messages
		gpos = current_messages.global_position_int;
		adsb = current_messages.adsb_vehicle_t;

		//Update our aircraft position (X, Y) A
		ourAircraft.lat[2] = ourAircraft.lat[1];
		ourAircraft.lon[2] = ourAircraft.lon[1];

		ourAircraft.lat[1] = ourAircraft.lat[0];		
		ourAircraft.lon[1] = ourAircraft.lon[0];

		ourAircraft.lat[0] = gpos.lat;
		ourAircraft.lon[0] = gpos.lon;
			

		//Update other aircraft position (X, Y) B
		otherAircraft.lat[2] = otherAircraft.lat[1];
		ourAircraft.lon[2] = otherAircraft.lon[1];

		otherAircraft.lat[1] = otherAircraft.lat[0];		
		ourAircraft.lon[1] = otherAircraft.lon[0];

		//otherAircraft.lat[0] = adsb.lat;
		//otherAircraft.lon[0] = adsb.lon;


		otherAircraft.lat[0] = 34.0590949;
		otherAircraft.lon[0] = -117.8210655;
			
		//Update our aircraft velocity (v) A
		ourAircraft.velocityX[1] = ourAircraft.velocityX[0];
		ourAircraft.velocityY[1] = ourAircraft.velocityY[1];		

		ourAircraft.velocityX[0] = gpos.vx;
		ourAircraft.velocityY[0] = gpos.vy;

		//Update other aircraft velocity (v) B
		otherAircraft.velocityX[1] = otherAircraft.velocityX[0];
		otherAircraft.velocityY[1] = otherAircraft.velocityY[1];		

		//otherAircraft.velocityX[0] = adsb.hor_velocity;
		//otherAircraft.velocityY[0] = adsb.ver_velocity;


		//Hard coded velocity
		otherAircraft.velocityX[0] = 0;
		otherAircraft.velocityY[0] = 0;


		//Update our aircraft acceleration (acc) A
		ourAircraft.xAcc = (ourAircraft.velocityX[0] - ourAircraft.velocityX[1]);//xAcc
		ourAircraft.yAcc = (ourAircraft.velocityY[0] - ourAircraft.velocityY[1]);//yAcc

		//Update our aircraft acceleration (acc) B
		otherAircraft.xAcc = (otherAircraft.velocityX[0] - otherAircraft.velocityX[1]);//xAcc
		otherAircraft.yAcc = (otherAircraft.velocityY[0] - otherAircraft.velocityY[1]);//yAcc

		//Predict
		collision = CA_Predict_Loiter(ourAircraft, otherAircraft);
		
	
		//Avoid if necessary
		if (collision.collisionDetected == true && AVOID_DELAY == 0) {
			
			AVOID_DELAY = 3;

			//printf("Time to collision from in here: %f\n", collision.timeToCollision);
			CA_Avoid(ourAircraft, otherAircraft, collision);
	
		}
		
		printf("Distance between aircraft: %f\n", gpsDistance(ourAircraft.lat[0], ourAircraft.lon[0], otherAircraft.lat[0], otherAircraft.lon[0]));
		if (AVOID_DELAY > 0) {
			AVOID_DELAY--;
			printf("Waiting for aircraft to avoid...\n");
		}
 
		sleep (1);

	}

	CA_status = false;

}


predictedCollision
Autopilot_Interface::
CA_Predict_Loiter(aircraftInfo & aircraftA, aircraftInfo & aircraftB) {

/*
struct Position {
	//Initialized to 0 to see if it has been initialized before. I'm using 0 as null value.
	Position() : Longitude(0), Latitude(0), Altitude(0), timestamp(0) {};
	double Latitude;
	double Longitude;
	double Altitude;

	uint8_t timestamp;
};


struct AircraftInfo {
	float lat[3];
	float lon[3];
	float alt[3];
	float velocityX[2];
	float velocityY[2];
	float futureDistx[3];
	float futureDisty[3];
	float xAcc;
	float yAcc;
	float hdg;
	float safetyBubble;

	uint8_t priority;

};

#define DELTA_T	0.5 /* change in time per timestep / // Old code unsure if necessary 

#define MAX_TIMESTEP	20.0 /* maximum timestep considered / // Old code unsure if necessary 

#define GPS_ERROR	10.0 /* gps error in meters / // Old code unsure if necessary 

#define AVG_VELOCITY	17.88 /* average aircraft velocity in m/s  // Old code unsure if necessary 

#define PI 3.14159265359

#define TO_RADIANS  PI / 180;

/* in meters /
#define CIRC_OF_EARTH	40075160.0
#define RADIUS_EARTH	6378037.0






float gpsHeading(const Position & previous, const Position & current);

/*Old code unsure if necessary/
Position gpsOffset(Position start, double dlatitude, double dlongitude);


struct SafetyBubble {
	/* circle used as buffer zone for possible collisions (i.e. hitbox)
	Position position;

	/*Radius in meters/
	double radius;
};

struct CollisionPoint 
{
	float lat;
	float lon;
	float alt;
	float time;
	float approachAngle;
};
*/



Position senseCollision(const AircraftInfo &, const Position &, const double &, const AircraftInfo &);

bool senseCollisionTrue(const AircraftInfo &, const Position &, const double &, const AircraftInfo &);

vector<AircraftInfo> predictPath(const AircraftInfo &);

string returnDirection(const AircraftInfo &, const Position &);

vector<AircraftInfo> predictPathCircular(const AircraftInfo &, const Position &, const double &);

void updateAircraftInfo(AircraftInfo & aircraft);

Position avoidCollisionLoiter(const AircraftInfo &, const AircraftInfo &);

Position avoidCollision(const AircraftInfo & otherPlaneInfo, const AircraftInfo & ourPlaneInfo, const SafetyBubble &, const SafetyBubble &, const int &);
	//ourPlane is loitering/avoiding

	AircraftInfo ourPlane, otherPlane;
	
	int counter = 0;

	Position center; 
	double radius;

	//auto takeoff
	//start predicting

	while(1==1)
	{
	//updateAircraftInfo(ourPlane);
	//updateAircraftInfo(otherPlane);
	counter++;
	if (counter > 3) 
	{
		senseCollision(ourPlane, center, radius, otherPlane);
		if (senseCollisionTrue(ourPlane, center, radius, otherPlane) == true) 
			cout << "\nGo to waypoint (" << senseCollision(ourPlane, center, radius, otherPlane).Latitude << ", " << senseCollision(ourPlane, center, radius, otherPlane).Longitude << ")";
		else
			continue;
	}
	} 
	
	system("pause");
	return 0;
}


Position senseCollision(const AircraftInfo & ourAirplane, const Position & cent, const double & r,
	const AircraftInfo & otherAirplane) {
	/*Null position has 0 values*/
	Position null;

	//If our airplane has priority, then we do not try to avoid

	vector<AircraftInfo> predictionOur;
	vector<AircraftInfo> predictionTheir;

	//get the predicted path for all planes
	predictionOur = predictPathCircular(ourAirplane, cent, r);
	predictionTheir = predictPath(otherAirplane);

	CollisionPoint colpt;

	for (int i = 0; i < predictionTheir.size(); i++) {

		//Create the safety bubble around the predictive positions.
		//The radius grows with each prediction because of the error in predicting positions
		SafetyBubble bubble1, bubble2;

		Position pos1, pos2; 
		pos1.Latitude = predictionOur[i].lat[0]; pos1.Longitude = predictionOur[i].lon[0];
		pos2.Latitude = predictionTheir[i].lat[0]; pos2.Longitude = predictionTheir[i].lon[0];

		// 3 meters + .5 meters after each timestep
		bubble1.radius = i*0.5 + 50; /*!!!!THE RADIUS OF THE BUBBLE IS STILL UNKNOWN!!!*/
		bubble2.radius = i*0.5 + 50;

		double distance = gpsDistance(pos1, pos2);		

		//If the predictive distance between the airplanes is less than the radius of the bubble
		if (distance < (bubble1.radius + bubble2.radius)) {
			cout << "Collision is predicted at (" << pos1.Latitude << ", " << pos1.Longitude << "). Go to avoid.";
			colpt.lat = pos1.Latitude; colpt.lon = pos1.Longitude; colpt.alt = pos1.Altitude; colpt.time = i;
			return avoidCollisionLoiter(ourAirplane, predictionTheir.at(i));
		}
		else {
			return null;
			cout << "No collision predicted";
		}

	}

}

bool senseCollisionTrue(const AircraftInfo & ourAirplane, const Position & cent, const double & r,
	const AircraftInfo & otherAirplane) {
	/*Null position has 0 values*/
	Position null;

	//If our airplane has priority, then we do not try to avoid

	vector<AircraftInfo> predictionOur;
	vector<AircraftInfo> predictionTheir;

	//get the predicted path for all planes
	predictionOur = predictPathCircular(ourAirplane, cent, r);
	predictionTheir = predictPath(otherAirplane);


	for (int i = 0; i < predictionTheir.size(); i++) {

		//Create the safety bubble around the predictive positions.
		//The radius grows with each prediction because of the error in predicting positions
		SafetyBubble bubble1, bubble2;

		Position pos1, pos2;
		pos1.Latitude = predictionOur[i].lat[0]; pos1.Longitude = predictionOur[i].lon[0];
		pos2.Latitude = predictionTheir[i].lat[0]; pos2.Longitude = predictionTheir[i].lon[0];

		//3 meters + .5 meters after each timestep
		bubble1.radius = i*0.5 + 50; /*!!!!THE RADIUS OF THE BUBBLE IS STILL UNKNOWN!!!*/
		bubble2.radius = i*0.5 + 50;

		double distance = gpsDistance(pos1, pos2);

		//If the predictive distance between the airplanes is less than the radius of the bubble
		if (distance < (bubble1.radius + bubble2.radius))
			return true;
		else
			return false;

	}

}

vector<AircraftInfo> predictPath(const AircraftInfo & airplane) {
	/*Time it takes to update position*/ /*I AM UNSURE OF WHAT THIS WILL END UP BEING*/
	double time = 1;

	/*Needed to find the x and y vector velocity. */
	Position calcLongVel, calcLatVel;
	calcLongVel.Latitude = airplane.lat[1];
	calcLongVel.Longitude = airplane.lon[2];

	calcLatVel.Latitude = airplane.lat[2];
	calcLatVel.Longitude = airplane.lon[2];

	Position veryLastPosition, lastPosition, currentPosition;
	currentPosition.Latitude = airplane.lat[0]; currentPosition.Longitude = airplane.lon[0];
	lastPosition.Latitude = airplane.lat[1]; lastPosition.Longitude = airplane.lon[1];
	veryLastPosition.Latitude = airplane.lat[2]; veryLastPosition.Longitude = airplane.lon[2];


	//calculate past velocity. Veloctiy between positions: last and veryLast
	double xVel1 = (gpsDistance(lastPosition, calcLongVel)) / time;
	double yVel1 = (gpsDistance(lastPosition, calcLatVel)) / time;

	calcLongVel.Latitude = currentPosition.Latitude;
	calcLongVel.Longitude = lastPosition.Longitude;

	calcLatVel.Latitude = lastPosition.Latitude;
	calcLatVel.Longitude = currentPosition.Longitude;

	//current velocity. Velocity between postions: current and last
	double xVel2 = (gpsDistance(currentPosition, calcLongVel)) / time;
	double yVel2 = (gpsDistance(currentPosition, calcLatVel)) / time;

	if (lastPosition.Longitude - veryLastPosition.Longitude < 0) {
		xVel1 = -xVel1;
	}
	if (lastPosition.Latitude - veryLastPosition.Latitude < 0) {
		yVel1 = -yVel1;
	}

	if (currentPosition.Longitude - lastPosition.Longitude < 0) {
		xVel2 = -xVel2;
	}
	if (currentPosition.Latitude - lastPosition.Latitude < 0) {
		yVel2 = -yVel2;
	}

	//calculate the acceleration
	// a = (v0 - v1) / t; Acceleration equation
	double xAcc2 = (xVel2 - xVel1) / time;
	double yAcc2 = (yVel2 - yVel1) / time;

	//Distance between the current postion and the future position
	//d = v*t + (.5)a*t^2; displacement equation
	double deltaX = xVel2*time + 0.5*(xAcc2)*pow(time, 2);
	double deltaY = yVel2*time + 0.5*yAcc2*pow(time, 2);

	//Holds the 10 aircraft opjects that will have predictive positions and headings
	vector<AircraftInfo> future(20);

	//The first aircraft object has the same positions and heading as the current airplane.
	//This way we can use it in the for loop
	future[0].lat[0] = airplane.lat[0]; future[0].lat[1] = airplane.lat[1]; future[0].lat[2] = airplane.lat[2];
	future[0].lon[0] = airplane.lon[0]; future[0].lon[1] = airplane.lon[1]; future[0].lon[2] = airplane.lon[2];
	future[0].alt[0] = airplane.alt[0]; future[0].alt[1] = airplane.alt[1]; future[0].alt[2] = airplane.alt[2];
	future[0].hdg = airplane.hdg;

	const float NORTH = 0;
	const float EAST = 90;
	const float SOUTH = 180;
	const float WEST = 270;

	for (int t = 1; t < future.size(); t++) {
		//Using the kinematic equation [displacement = v*t + (.5)a*t^2] and then adding the current position, 
		//we can predict the future position for t timesteps. 
		future[t].lat[0] = meterDisplacement(xVel2*t + 0.5*(xAcc2)*pow(t, 2),
			yVel2*t + 0.5*yAcc2*pow(t, 2), currentPosition).Latitude;
		future[t].lon[0] = meterDisplacement(xVel2*t + 0.5*(xAcc2)*pow(t, 2),
			yVel2*t + 0.5*yAcc2*pow(t, 2), currentPosition).Longitude;

		Position futureCurrentPosition, futureLastPosition;
		futureCurrentPosition.Latitude = future[t].lat[0];
		futureCurrentPosition.Longitude = future[t].lon[0];
		futureLastPosition.Latitude = future[t - 1].lat[0];
		futureLastPosition.Longitude = future[t - 1].lon[0];

		/*Rest of conditionals is for predicting heading.
		MAY NOT WORK RECOMMEND POSSIBLE REWRITE*/
		if (xVel2 + (xAcc2 *t) == 0) {
			if (yAcc2 > 0) {
				future[t].hdg = EAST;
			}
			else {
				future[t].hdg = WEST;
			}
		}
		else if (yVel2 + (yAcc2 *t) == 0) {
			if (xAcc2 > 0) {
				future[t].hdg = NORTH;
			}
			else {
				future[t].hdg = SOUTH;
			}
		}
		else {
			float h = gpsHeading(futureCurrentPosition, futureLastPosition);
			future[t].hdg = h;

		}
	}
	return future;

}

string returnDirection(const AircraftInfo & airplane, const Position & center) 
{
	string direction;
	if (((airplane.lon[1] - center.Longitude)*(airplane.lat[0] - center.Latitude) - (airplane.lat[1] - center.Latitude)*(airplane.lon[0] - center.Longitude)) > 0)
	{
		direction = "counter";
		return direction;
	}
	else
	{
		direction = "clock";
		return direction;
	}

}

vector<AircraftInfo> predictPathCircular(const AircraftInfo & airplane, const Position & center, const double & r) {
	//Time it takes to update position/timestep
	double time = 1;
	string direction = "counter";

	double velocity = sqrt(airplane.velocityX[0] * airplane.velocityX[0] + airplane.velocityY[0] * airplane.velocityY[0]);

	//angular velocity	
	double angVel = velocity / r;


	direction = returnDirection(airplane, center);

	//Holds the 20 aircraft opjects that will have predictive positions and headings
	vector<AircraftInfo> future(20);

	//The first aircraft object has the same positions and heading as the current airplane.
	//This way we can use it in the for loop
	future[0].lat[0] = airplane.lat[0]; future[0].lat[1] = airplane.lat[1]; future[0].lat[2] = airplane.lat[2];
	future[0].lon[0] = airplane.lon[0]; future[0].lon[1] = airplane.lon[1]; future[0].lon[2] = airplane.lon[2];
	future[0].alt[0] = airplane.alt[0]; future[0].alt[1] = airplane.alt[1]; future[0].alt[2] = airplane.alt[2];
	future[0].hdg = airplane.hdg;

	for (int t = 1; t < future.size(); t++) {
		//Using the kinematic equation [displacement = v*t + (.5)a*t^2] and then adding the current position, 
		//we can predict the future position for t timesteps.

		double theta;

		if (direction == "clock") {
			future[t].hdg = future[0].hdg - (angVel*t)*(180 / PI);
			theta = (future[t].hdg + 90)*(PI / 180);
		}
		else if (direction == "counter") {
			future[t].hdg = future[0].hdg + (angVel*t)*(180 / PI);
			theta = (future[t].hdg - 90)*(PI / 180);
		}

		if (future[t].hdg < 0)
			while (future[t].hdg < 360)
				future[t].hdg += 360;
		else if (future[t].hdg >= 360)
			while (future[t].hdg >= 360)
				future[t].hdg -= 360;

		future[t].lat[0] = gpsOffset(center, r*sin(theta), r*cos(theta)).Latitude;
		future[t].lon[0] = gpsOffset(center, r*sin(theta), r*cos(theta)).Longitude;

	}

	return future;

}

Position meterDisplacement(const double & deltaX, const double & deltaY, const Position & b) {
	//coordinate offset in Radians
	float deltaLat = (deltaY / RADIUS_EARTH);
	float deltaLong = deltaX / (RADIUS_EARTH * cos(b.Latitude * PI / 180));

	Position newPosition;
	newPosition.Latitude = b.Latitude + (deltaLat * (180 / PI));
	newPosition.Longitude = b.Longitude + deltaLong * ((180 / PI));
	return newPosition;
}

Position gpsOffset(Position start, double dlatitude, double dlongitude)
{
	Position toReturn;

	toReturn.Latitude = start.Latitude + dlatitude*(360.0 / CIRC_OF_EARTH);
	toReturn.Longitude = start.Longitude + dlongitude*(360.0 / CIRC_OF_EARTH)*(1 / cos(start.Latitude*(PI / 180.0)));

	toReturn.Altitude = start.Altitude;

	return toReturn;
}

double gpsDistance(const Position & a, const Position & b) {

	double aLatRad = a.Latitude*TO_RADIANS;
	double bLatRad = b.Latitude*TO_RADIANS;
	double deltaLat = (b.Latitude - a.Latitude) * TO_RADIANS;
	double deltaLong = (b.Longitude - a.Longitude)*TO_RADIANS;

	double var = sin(deltaLat / 2) * sin(deltaLat / 2) + (cos(aLatRad) * cos(bLatRad)) * (sin(deltaLong / 2) * sin(deltaLong / 2));
	double c = 2 * atan2(sqrt(var), sqrt(1 - var));

	return RADIUS_EARTH * c;

}

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

void updateAircraftInfo(AircraftInfo & aircraft) {
	//update with adsb and gpos stuff
}

Position avoidCollisionLoiter(const AircraftInfo & ourPlaneInfo, const AircraftInfo & otherPlaneInfo)
{
	Position ourPlanePosition;
	ourPlanePosition.Latitude = ourPlaneInfo.lat[0];
	ourPlanePosition.Longitude = ourPlaneInfo.lon[0];
	ourPlanePosition.Altitude = ourPlaneInfo.alt[0];
	//string direction = returnDirection(ourPlaneInfo, center);
	float rH1 = (otherPlaneInfo.hdg - 90) - ourPlaneInfo.hdg;
	float rH2 = (otherPlaneInfo.hdg + 90) - ourPlaneInfo.hdg;
	if(rH1 < rH2) 
		return meterDisplacement(300 * cos((otherPlaneInfo.hdg - 90)*(PI / 180)), 300 * sin((otherPlaneInfo.hdg - 90)*(PI / 180)), ourPlanePosition);
	else 
		return meterDisplacement(300 * cos((otherPlaneInfo.hdg + 90)*(PI / 180)), 300 * sin((otherPlaneInfo.hdg + 90)*(PI / 180)), ourPlanePosition);
}

Position avoidCollision(const AircraftInfo & otherPlaneInfo, const AircraftInfo & ourPlaneInfo, const SafetyBubble & otherPlaneBubble,
	const SafetyBubble & ourPlaneBubble, const int & timeToCollision) {

	bool turnRight; // True if plane2 has to turn right and false if plane2 has to turn left

					// If heading1 >= 180 
	if (otherPlaneInfo.hdg >= 180) {

		// turnRight if  [ (heading1 - 180) <= heading2 <= heading1 ]
		turnRight = (((otherPlaneInfo.hdg - 180) <= ourPlaneInfo.hdg) && (ourPlaneInfo.hdg <= otherPlaneInfo.hdg));

	}
	// heading1 < 180 
	else {

		// turnLeft if [ heading1 < heading2 < (heading1 + 180) ]
		turnRight = !((otherPlaneInfo.hdg < ourPlaneInfo.hdg) && (ourPlaneInfo.hdg < (otherPlaneInfo.hdg + 180)));
		// Therefore, do not turn right if you have to turn left 
	}

	// This will determine how large the AVOID_BUFFER will be
	const int arbitraryConstant = 10; /*!!!!! THIS IS YET TO BE DETERMINED!!!! */

									  //This will allow the distance of the new waypoint to be smaller or larger depending on the timeToCollision
	const double AVOID_BUFFER = arbitraryConstant / (timeToCollision * otherPlaneBubble.radius);

	// Length k = distance needed between avoidWaypoint and B 
	double k = otherPlaneBubble.radius + AVOID_BUFFER;

	Position avoidWaypoint; // Referred to as C in pseudocode

	Position A;
	A.Latitude = ourPlaneInfo.lat[0]; A.Longitude = ourPlaneInfo.lon[0];
	Position B = otherPlaneBubble.position;

	// Guard against division by zero when finding slopes
	/*!!!!!Increment value(.00000001) IS YET TO BE DETERMINED!!!!!*/
	if (B.Longitude - A.Longitude == 0) B.Longitude += .00000001;
	if (B.Latitude - A.Latitude == 0) B.Latitude += .00000001;

	// Slope of line AB
	double slopeAB = (B.Latitude - A.Latitude) / (B.Longitude - A.Longitude);

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

	return collisionPoint;
}







void 
Autopilot_Interface::
CA_Avoid( aircraftInfo & aircraftA, aircraftInfo & aircraftB, predictedCollision &collision)
{

	printf("Time to collision from in here: %f\n", collision.timeToCollision);
	printf("\nCOLLISION DETECTED\n");
	bool makeCircle = false;
	bool headOnCollision = false;
	bool approachingFromRight = false;
	mavlink_mission_item_t avoidWaypoint;
	bool turnRight = false;


	//----------------------------------------------------
	//Store mission waypoints inside a vector
	//----------------------------------------------------
	Request_Waypoints();



	//-------------------------------------------------------
	//Calculate avoid waypoint
	//-------------------------------------------------------

/* (our airplane)
         A---____
         |       -----___
	 |               --B (other airplane)
         |                /
         |              /
         |             /
         |        ^   /
         |       /  /
         |      K  /
         |     /  /
         |    v /
         |     /
         |    /
         |  /
         | /
         |/
         C  (Avoid waypoint)  */



	//This will allow the distance of the new waypoint to be smaller or larger depending on the timeToCollision
		AVOID_BUFFER = avoidBufferSize + (aircraftB.safetyBubble / collision.timeToCollision);
		if (collision.timeToCollision < 1) AVOID_BUFFER = avoidBufferSize;
		printf("timeToCollision: %f\n", collision.timeToCollision);
		printf("avoid buffer size: %i\n", avoidBufferSize);
		

	// Length k = distance needed between avoidWaypoint and B 
		float k = aircraftB.safetyBubble + AVOID_BUFFER;

	// Guard against division by zero when finding slopes
	//!!!!!Increment value(.00000001) IS YET TO BE DETERMINED!!!!!
		if (aircraftB.lon[0] - aircraftA.lon[0] == 0) aircraftB.lon[0] += .00000001; 
		if (aircraftB.lat[0] - aircraftA.lat[0] == 0) aircraftB.lat[0] += .00000001;

	// Slope of line AB
		double slopeAB = (aircraftB.lon[0] - aircraftA.lon[0]) / (aircraftB.lat[0] - aircraftA.lat[0]);

	// Slope of line BC, such that C = avoidWaypoint.
		double slopeBC = -1 / slopeAB;

	// xDisplacement = k * ( 1 / ((1 + slopeBC^2)^(1/2)) )
		double xDisplacement = k * (1 / sqrt(1 + (slopeBC*slopeBC)));

	// yDisplacement = k * ( slopeBC / ((1 + slopeBC^2)^(1/2)) )
		double yDisplacement = k * (slopeBC / sqrt(1 + (slopeBC*slopeBC)));
		

		float rH = collision.relativeHeading;
		printf("X displacement: %f Y displacement: %f\n", xDisplacement, yDisplacement);

		
			if (rH <= 20 || rH >= 340) 
				{
				printf("MAKE A CIRCLE: ");
				makeCircle = true;
				}
		
			else if (170 <= rH && rH <= 190)
				{
				printf("HEAD ON COLLISION: ");
				headOnCollision = true;
				}

	
			else if (rH < 360 && rH > 180)
				{
				printf("Plane 2 is approaching from the right: ");
				approachingFromRight = true;
				}

			else
				{
				printf("Plane 2 is approaching from the left: ");
				}


		

		if (makeCircle == true && approachingFromRight == true && headOnCollision == false) {
			printf("Rotate CounterClockwise\n\n");
			avoidWaypoint = NewAvoidWaypoint(-xDisplacement, -yDisplacement, aircraftA);
		}

			else if (makeCircle == true && approachingFromRight == false && headOnCollision == false) {
				printf("Rotate Clockwise\n\n");
				avoidWaypoint = NewAvoidWaypoint(xDisplacement, yDisplacement, aircraftA);
			}


			else if (approachingFromRight == true && headOnCollision == false) {
				turnRight = true;
				printf("Turn Right\n");
				avoidWaypoint = NewAvoidWaypoint(xDisplacement, yDisplacement, aircraftA);
				}


			else if (approachingFromRight == false && headOnCollision == false) {
				turnRight = false;
				printf("Turn Left\n\n");
				avoidWaypoint = NewAvoidWaypoint(-xDisplacement, -yDisplacement, aircraftA);
				}


			else if (headOnCollision == true) {
				printf("Turn Right\n\n");
				avoidWaypoint = NewAvoidWaypoint(xDisplacement, yDisplacement, aircraftA);
			}

		
		//printf(avoidWaypoint.Lat << avoidWaypoint.Long << "\n\n";
		


		//-----------------------------------------------------------------------
		//Insert waypoint
		//-----------------------------------------------------------------------

		//printf("Current mission size: %lu\n", currentMission.size());


			currentMission.push_back(currentMission[3]); //Takes first element and copies it to last for dummy data
			int endVal = currentMission.size() - currentWaypoint;
			int i;
			

				for (i = 0; i < endVal; i++) {
					//printf("\nLatitude before %f\n", currentMission.rbegin()[i].x);
					currentMission.rbegin()[i] = currentMission.rbegin()[(i+1)]; //Shifts waypoints over until the inserted waypoint is reached
					currentMission.rbegin()[i].seq = currentMission.size() - (i+1);
					//printf("Latitude after %f\n", currentMission.rbegin()[i].x);
				}

			avoidWaypoint.z = currentMission[endVal].z; //I chose endVal because it is guaranteed to always be there
			avoidWaypoint.seq = currentWaypoint;
			currentMission[currentWaypoint] = create_waypoint(avoidWaypoint.x, avoidWaypoint.y, currentMission[endVal].z, currentWaypoint, 15);


			//Write avoid waypoint within mission
			write_waypoints(currentMission);

			printf("\n\n");

			//Tell the aircraft to go to the waypoint
			setCurrentWaypoint(currentWaypoint);

		printf("Collision point created\n");
}


