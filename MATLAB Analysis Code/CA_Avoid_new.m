function [ lat, lon ] = CA_Avoid( aircraftA, aircraftB, timeToCollision)
% Version of collision avoidance function to test and debug
%
%
% INPUT:
%        aircraftA/B: struct
%        1. lat: latitude of current poition
%        2. lon: longitude of current poition   
%        3. safetyBubble: radius of safety bubble at predicted collision
%        point
%
    
    %aircraftInfo struct (code uses this)
    
    % aircraftInfo() {
    % 	//Initializes all values to zero
    % 	//Descriptions are in the aircraftInfo struct
    % 	lat [3] = {0};
    % 	lon [3] = {0};
    % 	alt [3] = {0};
    % 	velocityX [2] = {0};
    % 	velocityY [2] = {0};
    % 	xAcc = 0;
    % 	yAcc = 0;
    % 	futureDistx [3] = {0};
    % 	futureDisty [3] = {0};
    % 	Hdg = 0;
    % 	safetyBubble = 10;  //Roughly 60 ft
    %   priority = 0;


    avoidBufferSize = 100; % From autopilot_interface.h
%     makeCircle = 0;
%     headOnCollision = 0;
%     approachingFromRight = 0;
% % 	mavlink_mission_item_t avoidWaypoint;
%     turnRight = false;

% 	//----------------------------------------------------
% 	//Store mission waypoints inside a vector
% 	//----------------------------------------------------
% 
% 	Request_Waypoints();

% 	//-------------------------------------------------------
% 	//Calculate avoid waypoint
% 	//-------------------------------------------------------


% 
%  (our airplane)
% 
%          A-----------------B (other airplane)
%          |                /
%          |              /
%          |             /
%          |        ^   /
%          |       /  /
%          |      K  /
%          |     /  /
%          |    v /
%          |     /
%          |    /
%          |  /
%          | /
%          |/
%          C  (Avoid waypoint) 

RADIUS_EARTH = 6371e3; % meters

latA = aircraftA(1);
lonA = aircraftA(2);
buffA = aircraftA(3);

latB = aircraftB(1);
lonB = aircraftB(2);
buffB = aircraftB(3);

	%//This will allow the distance of the new waypoint to be smaller or larger depending on the timeToCollision
		AVOID_BUFFER = avoidBufferSize + (buffB / timeToCollision);
        
		if (timeToCollision < 1) 
            AVOID_BUFFER = avoidBufferSize;
        end

%       Length k = distance needed between avoidWaypoint and B (meters)
		k = buffB + AVOID_BUFFER;
        % Angular distance
        Ad = k/RADIUS_EARTH;
		%addToFile(convertToString(k), "k: distance between waypoint and B");



	% Guard against division by zero when finding slopes
	%!!!!!Increment value(.00000001) IS YET TO BE DETERMINED!!!!!
		if (lonB - lonA == 0) 
            lonB = lonB + .00000001;
        end
		if (latB - latA == 0) 
            latB = latB + .00000001;
        end
        
	% Bearing from A to B: from https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
        dLon = lonB - lonA;
        dLat = latB - latA;
        x = cosd(latB) * sind(dLon);
        y = cosd(latA) * sind(latB) - sind(latA) * cosd(latB) * cosd(dLon);
		
        
        % Special cases in dealing with NED frame
            bearingAB = atan2d(y,x);
            if and(dLon < 0, dLat > 0) || and(dLon > 0, dLat < 0)
                bearingAB = bearingAB + 180;
            end
        
	% Slope of line BC: right now it is simply a 80 degree turn
		bearingBC = bearingAB + 80;

	% displacement in meters of new waypoint in x direction (North/South)
		lat = asind(sind(latA) * cos(Ad) + cosd(latA)* sin(Ad) * cosd(bearingBC));
        % Longitude displacement
	    lon_disp = atan2d( sind(bearingBC) * sin(Ad) * cosd(latA), cos(Ad) - sind(latA) * sind(latB));
		lon = lonA + lon_disp;

        
		% insert_waypoint( avoidWaypoint, currentWaypoint);		
        % [ lat , lon ] = NewAvoidWaypoint(xDisplacement, yDisplacement, [aircraftA(1),aircraftB(2)]);
        
        end
