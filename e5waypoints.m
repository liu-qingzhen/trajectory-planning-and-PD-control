% Create sample waypoint data for trajectory generation
% NOTE: Modify this script with your own rigid body tree and 
%       trajectory reference points
%       (We recommend saving a copy of this file)
%
% Copyright 2019 The MathWorks, Inc.

%% Common parameters
% Rigid Body Tree information
rootDir = fileparts(which(mfilename));
addpath(genpath('matlab'),genpath('simulink'),genpath('utilities'));

load e5
eeName='elfin_end_link';
numJoints = numel(e5.homeConfiguration);
ikInitGuess=[-0.5276    0.7811   -1.7791   -0.0000    2.5600    0.5276];
% ikInitGuess=e5.randomConfiguration;
tform = getTransform(e5,ikInitGuess,eeName,'world');
endeff=tform2trvec(tform)
%%


% Positions (X Y Z)
waypoints = endeff' + ... 
[0 0 0.2 ; -0.1 0.2 0.4 ; -0.2 0 0.1 ; -0.1 -0.2 0.4 ; 0 0 0.2]'         
orientations = [0     0    0;
                pi/8  0    0; 
                0    pi/2  0;
               -pi/8  0    0;
                0     0    0]';   
            
                       
% Array of waypoint times
waypointTimes = 0:4:16;
% Trajectory sample time
ts = 0.2;
trajTimes = 0:ts:waypointTimes(end);

%% Additional parameters

% Boundary conditions (for polynomial trajectories)
% Velocity (cubic and quintic)
waypointVels = 0.1 *[ 0  1  0;
                     -1  0  0;
                      0 -1  0;
                      1  0  0;
                      0  1  0]';

% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));

% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;

