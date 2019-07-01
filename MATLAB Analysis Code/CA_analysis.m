% NGCP 2018-2019
% Code to analyze collision avoidance log output
% Created by Tristan Sherman
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~


% !!NOTE!! to use wmmarker the MATLAB mapping toolbox must be installed




%Clear variables
% if exist('log','var')
%     clearvars -except log
% else
    clear
%end
clc
wmclose %Close map

% Pick the file name to analyze
filename = 'logging_file_Test2.txt';
[log, collisionPoints] = parseCAlog(filename); %Parse log file into variable
obstacleLat = 34.059249;
obstacleLon = -117.820915;




% Choose which timestep you want to display
position = 136;
position = (position * 2) - 1; % Column of desired position




% Store the three GPS positions provided by the GPS
lastPoint= [log(1,position), log(1,position+1)];
secondPoint = [log(2,position), log(2,position+1)];
currentPoint = [log(3,position), log(3,position+1)];




% for p = 65:length(log(1,:))
%     wmmarker(log(3,(p*2)-1),log(3,p*2));
% end


% The current lat, lon will be displayed as (0,0) in the plot
x(1) = 0;
y(1) = 0;
predictPointPosition = 5; % This is the row of the first predicted point
endPosition = 24; % The row of the final predicted point in log

for i = predictPointPosition:endPosition-10

 if log(i,position) ~=0
    nextPoint = [log(i,position), log(i,position+1)];
    wmmarker(log(i,position),log(i,position+1)); %Display point on web map
 end
 [dx,dy] = GPStoCartesian(currentPoint,nextPoint);
 x(i) = dx;
 y(i) = dy;
end

plot(x,y,'-o')
arrow([0,0],[log(4,position),log(4,position+1)], 'Length', 5) % This was a function downloaded from an online source
