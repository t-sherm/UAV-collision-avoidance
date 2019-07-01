function [dx,dy] = GPStoCartesian(currentPos, nextPos)
% NGCP 2018-2019
% Tristan Sherman
% Find x,y distance between gps coordinates and output them in cartesian
% frame

% INPUT: CURRENT POSITION - [lat, lon]
%        NEXT POS         - [lat, lon]

% Note: Input is in NED frame meaning North = +x and East = +y
% Output: distance in feet



% conversion to radians
TO_RADIANS = pi/180;
TO_FT = 1000 * 3.2808399;
RADIUS_EARTH = 6378.037; % km

currentLat = currentPos(1);
currentLon = currentPos(2);
nextLat = nextPos(1);
nextLon = nextPos(2);

% %Convert to radians
 latRad = currentLat * TO_RADIANS;
 lonRad = currentLon * TO_RADIANS;

%Find delta in positions
deltaLat = (nextLat - currentLat) * TO_RADIANS;
deltaLon = (nextLon - currentLon) * TO_RADIANS;

% Math from: http://www.movable-type.co.uk/scripts/latlong.html
%? is latitude, ? is longitude, R is earth’s radius (mean radius = 6,371km);
% note that angles need to be in radians to pass to trig functions!
% var R = 6371e3; // metres
% var ?1 = lat1.toRadians();
% var ?2 = lat2.toRadians();
% var ?? = (lat2-lat1).toRadians();
% var ?? = (lon2-lon1).toRadians();
% var a = Math.sin(??/2) * Math.sin(??/2) +
%         Math.cos(?1) * Math.cos(?2) *
%         Math.sin(??/2) * Math.sin(??/2);
% var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

% Using the above math to calculate dlon, we assume dlat = 0. This
% simplifies the equation
% Calculate x cartesian distance (longitude distance): no longer NED frame

varDlon = (cos(latRad))^2 * (sin(deltaLon/2))^2;
dx = 2 * atan2(sqrt(varDlon),sqrt(1-varDlon)) * RADIUS_EARTH * TO_FT;

if deltaLon < 0
    dx = -dx;
end
    
%Calculate dlat. Assume dlon = 0
varDlat = sin(deltaLat/2)^2;
dy = 2 * atan2(sqrt(varDlat),sqrt(1-varDlat)) * RADIUS_EARTH * TO_FT;

if deltaLat < 0
    dy = -dy;
end
%       
%     c = 2 * atan2(sqrt(var), sqrt(1 - var));
%   dx = deltaX * TO_FT;
%   dy = deltaY * TO_FT;
end
