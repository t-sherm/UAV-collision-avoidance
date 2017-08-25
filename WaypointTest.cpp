/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *	     Tristan Sherman, <tristan.m.sherman@gmail.com>
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
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 * @author Tristan Sherman <tristan.m.sherman@gmail.com>
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "CA.h"
#include <fstream>


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	int baudrate = 57600;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);


	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
	Serial_Port serial_port(uart_name, baudrate);


	/*
	 * Instantiate an autopilot interface object
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
	 *
	 */
	Autopilot_Interface autopilot_interface(&serial_port);

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	serial_port.start();

	autopilot_interface.messages_to_read.read_global_position_int = true;
	autopilot_interface.messages_to_read.read_heartbeat = true;
 	autopilot_interface.messages_to_read.read_attitude = true;

	autopilot_interface.start();

	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */
	commands(autopilot_interface);


	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	serial_port.stop();


	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api)
{

	//Initializations
	int mission;
	bool ask = true; //Stays true until a valid mission is created
	Mavlink_Messages messages = api.current_messages;
	//messages = api.current_messages;
	mavlink_global_position_int_t gpos = messages.global_position_int;
	vector<mavlink_mission_item_t> waypoints;



while (ask == true) {

	printf("Which mission would you like to perform?");
	printf(	"\n1. Create giant line across U.S."); 
	printf(	"\n2. Create square");
	printf(	"\n3. Make a set of arbitrary waypoints");
	printf(	"\n4. Replace waypoint 3 while flying");
	printf(	"\n5. Insert waypoint between two others");
	printf( "\n6. Change next waypoint");
	printf( "\nMission: ");
	cin >> mission;
	
	switch(mission) {

		case 1: {
		printf("\nYou have chosen mission %i\n", mission);
		printf("This function will add new waypoints linearly in order to the pixhawk.\n");
		//Once the aircraft reaches the first waypoint another will be generated and this will be continued indefinitely
		//All the waypoints will be saved
		
			int check = 1;
			int k = 1;
		
			waypoints.push_back(api.create_waypoint(0,0,0,0)); //Dummy data that is required* but not used

			while (check == 1) {
				
				waypoints.push_back(api.create_waypoint(32 + k, -117 + k,100,k,20 + (k/5)));
 				
				api.write_waypoints(waypoints);


				ofstream myfile;
  				myfile.open ("log.txt");
  				myfile << "\n" << k;
  				myfile.close();
					
				printf("Waypoint %i created\n", k);
				
				printf("Continue? y=1 n=2  ");

				cin >> check;
				
				k++;

			}


		ask = false;
		break;
		}

		case 2: {
		printf("\nYou have chosen mission %i\n", mission);

			//Initialize top left corner of square
			waypoints.push_back(api.create_waypoint(0,0,0,0)); //Dummy data that is required* but not used
			waypoints.push_back(api.create_waypoint(34.0592171, -117.8217709,100,1,10));//write new waypoint
			api.write_waypoints(waypoints);

			//Initialize counters
			int contin = 1;
			int counter = 0;
			bool atWaypoint = false;
			
			//Initialize current position
			mavlink_global_position_int_t gpos = messages.global_position_int;

			bool waypointDist;

			while (contin == 1) {

				api.write_waypoints(waypoints);		

				//Wait for the aircraft to be within the waypoint
				while (!atWaypoint) {

					waypointDist = api.gpsDistance(waypoints.back().x, waypoints.back().y, gpos.lat, gpos.lon); //Save distance from waypoint
					
					if( waypointDist < 10) {
						atWaypoint = true;
					}
				
				printf("Distance to waypoint: %d", waypointDist);
				sleep(1);

				}
				
				counter++;
				//printf("\nContinue 1 = ");
				//cin >> contin;


				//This makes waypoints in a square when the original waypoint is the top left corner in mission planner.
				switch (counter) {

					case 1:  {
					waypoints.push_back(api.createNewDisplacedWaypoint(30,0,waypoints.back()));
					atWaypoint = false;
					sleep(1);
					break;  
					}

					case 2:  {
					waypoints.push_back(api.createNewDisplacedWaypoint(0,-30,waypoints.back()));
					atWaypoint = false;
					sleep(1);
					break;
					}

					case 3: {
					waypoints.push_back(api.createNewDisplacedWaypoint(-30,0,waypoints.back()));
					atWaypoint = false;
					sleep(1);
					break;
					}

					case 4:  {
					waypoints.push_back(api.createNewDisplacedWaypoint(0,30,waypoints.back()));
					atWaypoint = false;
					sleep(1);
					break;
					}

					case 5:  {
					printf("Square created\n");
					contin = 2;
					break;
					}
				}//end switch
				

			}//end While




		ask = false;
		break;
		}//end case

		

		case 3: {
			printf("\nYou have chosen mission %i\n", mission);
		
			waypoints.push_back(api.create_waypoint(0,0,0,0)); //Dummy data that is required* but not used
			waypoints.push_back(api.create_waypoint(34.0592171, -117.8217709,100,1,10));//write new waypoint
		
			waypoints.push_back(api.createNewDisplacedWaypoint(30,0,waypoints.back()));
			waypoints.push_back(api.createNewDisplacedWaypoint(0,40,waypoints.back()));
			waypoints.push_back(api.createNewDisplacedWaypoint(-30,0,waypoints.back()));
			waypoints.push_back(api.createNewDisplacedWaypoint(-50,-10,waypoints.back()));


			api.write_waypoints(waypoints);

		ask = false;
		break;
		}


		case 4: {
		printf("\nYou have chosen mission %i\n", mission);
		
			int writeNum;
			//Create first mission
			waypoints.push_back(api.create_waypoint(0,0,0,0)); //Dummy data that is required* but not used
			waypoints.push_back(api.create_waypoint(34.0592171, -117.8217709,100,1,10));//write new waypoint
			waypoints.push_back(api.create_waypoint(34.0593326, -117.8214437,100,2,10));
			waypoints.push_back(api.create_waypoint(34.0594304, -117.8211299,100,3,10));
			waypoints.push_back(api.create_waypoint(34.0595482, -117.8208777,100,4,10));

			api.write_waypoints(waypoints);

			printf("Press any number to write waypoint ");
			cin >> writeNum;


			//Create intermediate waypoint
			waypoints[3] = api.create_waypoint(34.0589216, -117.8214410, 100, 3, 10);
			api.write_waypoints(waypoints);


			ask = false;
			break;
		}

		case 5: {
		printf("\nYou have chosen mission %i\n", mission);

			
			waypoints.push_back(api.create_waypoint(0,0,0,0)); //Dummy data that is required* but not used
			waypoints.push_back(api.create_waypoint(34.0592171, -117.8217709,100,1,10));//write new waypoint
			waypoints.push_back(api.create_waypoint(34.0593326, -117.8214437,100,2,10));
			waypoints.push_back(api.create_waypoint(34.0594304, -117.8211299,100,3,10));
			waypoints.push_back(api.create_waypoint(34.0595482, -117.8208777,100,4,10));

			api.write_waypoints(waypoints);
			
			int insertWaypoint;
			printf("What waypoint do you want the new one to be? ");
			cin >> insertWaypoint;


			int i = 0;
			 //The waypoint number you want to insert. If you want one between 2 and 3 type 3.
			int endVal = waypoints.size() - insertWaypoint;


			//-----------------------------------------------------------------------
			//Insert waypoint
			//-----------------------------------------------------------------------

			waypoints.push_back(waypoints[0]); //Creates new element

				for (i = 0; i <= endVal; i++) {
					waypoints.end()[-i] = waypoints.end()[-(i+1)]; //Shifts waypoints over until the inserted waypoint is reached
					waypoints.end()[-i].seq = waypoints.size() - (i);
				}

			waypoints[insertWaypoint] = api.create_waypoint(34.0589216, -117.8214410, 100, insertWaypoint, 10);

			api.write_waypoints(waypoints);

			ask = false;
		break;
		}


		case 6: {

			uint16_t nextWaypoint;

			printf("Enter the next waypoint for the aircraft to travel to: ");
			cin >> nextWaypoint;

			api.setCurrentWaypoint(nextWaypoint);

		ask = false;
		break;
		}

		default: {
		printf("Not a valid mission.\n\n");
		break;	
		}

	}//end big switch

}//end big while




	// --------------------------------------------------------------------------
	//   GET A MESSAGE
	// --------------------------------------------------------------------------
	//printf("READ SOME MESSAGES \n");

	
	// Heartbeat message
	//int i =0;


	//mavlink_heartbeat_t heart = messages.heartbeat;

	//printf("Type of MAV: %i \n", heart.mavlink_version );

	//sleep(1);
	

	//printf("\n");



	/*/Global Position


	for (i=0;i<2;i++){

	messages = api.current_messages;
	mavlink_global_position_int_t gpos = messages.global_position_int;

	printf("\nLatitude: %f \n", gpos.lat/1E7 );
	printf("Longitude: %f \n", gpos.lon/1E7 );
	printf("Altitude: %d \n", gpos.alt/1000);
	printf("Heading: %d \n", gpos.hdg/100);

	sleep(1);
	
	} 

	cout << "ADS-B Vehicle Information: \n";

	int i=0;

	for (i=0;i<2;i++){

	//messages = api.current_messages;
	mavlink_adsb_vehicle_t adsb = messages.adsb_vehicle_t;

	cout << "\nICAO Address: " << adsb.ICAO_address << "\n";
	printf("Latitude: %f \n", adsb.lat / 1E7);
	printf("Longitude: %f \n", adsb.lon / 1E7 );
	printf("Altitude: %f \n", adsb.altitude / 304.8 );

	usleep(10000);
	
	}

	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------
	*/
	return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


