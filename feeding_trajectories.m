function [] = feeding_trajectories()
% feeding_trajectories

%% Clear out old information to reduce problems with stale modules
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
HebiLookup.clearGroups();
pause(3);

%% Connect to physical robot
robotHardware = HebiLookup.newGroupFromNames('Robot A',{'base','shoulder', 'elbow', 'wrist 1', 'wrist 2'});
robotHardware.setCommandLifetime(2);

warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');
gains = load('capstone_gains.mat');
robotHardware.set('gains', gains.gains);
%% Get initial position
fbk = robotHardware.getNextFeedback();
initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robotHardware.startLog('file', fullfile(currentDir, 'robot_data'));

%% command frequency, in Hz
frequency = 100;

%% DH Parameters definition to be defined by student
link_lengths = [0;0.38;0.33;0;0];
link_twists = [pi/2;pi;pi;pi/2;0];
link_offsets = [0;0.09;0.07;0.09;0.17];
joint_angles = [0;0;0;pi/2;0];

dh_parameters = zeros(5,4);
dh_parameters(:,1) = link_lengths;
dh_parameters(:,2) = link_twists;
dh_parameters(:,3) = link_offsets;
dh_parameters(:,4) = joint_angles;
%% create object of Robot3D class
robot = Robot3D(dh_parameters); %Robot3d is the class defined for 3D robot

%% Move the robot to Home Position
disp('');
input('Put the robot in a safe position (spoon is horizontal)...','s');
% You need to modify the homePositioning as explained in writeup.
trajectory = homePositioning(robotHardware, 30, robot);

%% Define waypoints and time stamps for your trajectory
goal_positions = [];  % 0.4191; 0.0254;
bowl_location1 = [0.4191; 0.0254; 0; -pi/4; 0; 0] + [0;0;0;0;0;0];
bowl_location2 = [0.4191; 0.0254; 0; 0; -pi/2; 0] + [-0.04;0.04;0;0;0;0]; 
mouth_location = [0.135; 0.456; 0.43; 0; -pi/2; 0] + [0;-0.2;-0.06;0;0;0];
initial_position = trajectory(:,end);
offset = 0; % z-offset from ground (negative)
height = 0.045; % height of bowl
diameter = 0.12; % diameter of bowl

%% Run the trajectory generation and command trajectory

%%% Go to bowl location (Faster)
bowl = bowl_location1 + [0;0;height+0.05;0;0;0];
goal_positions = [goal_positions bowl];

%%% Scoop up some BB bullets
scoop1 = bowl_location1; 
scoop2 = bowl_location1; 
scoop3 = bowl_location2 + [0;0;height;0;0;0];
goal_positions = [goal_positions scoop1 scoop3 scoop3];

%%% Go to mouth location (CAREFUL - BULLETS ON SPOON)
goal_positions = [goal_positions mouth_location];

%%% Insert into mouth (CAREFUL - BULLETS ON SPOON)
forward = mouth_location + [0;0.22;0;0;0;0];
drop = mouth_location;
backward = mouth_location;
goal_positions = [goal_positions forward drop backward];

%%% Go back to bowl
bowl = bowl_location1 + [0;0;height+0.05;0;0;0];
goal_positions = [goal_positions bowl];

% Make waypoints
[row,col] = size(goal_positions);
col = col+1;
waypoints = zeros(5,col+1);
initial = [pi/4;pi/4;pi/4;pi/4;pi/4];
times = zeros(1,col+1);
waypoints(:,1) = initial_position;
for i = 1:col-1
    pitch = pi/3;
    if i == 2 % pitch = -pi/3
        previous = waypoints(:,i);
        waypoints(:,i+1) = previous + [0;0;0;-pitch;0];
    elseif i == 3 % roll = -pi/2
        previous = waypoints(:,i);
        waypoints(:,i+1) = previous + [0;0;0;0;-pi/2];
    elseif i == 7 % roll pi
        previous = waypoints(:,i);
        waypoints(:,i+1) = previous + [0;0;0;0;pi];
    else
        goal = goal_positions(:,i);
        goal = goal + [0;0;offset;0;0;0];
        waypoints(:,i+1) = robot.ik(initial,goal);
    end

    if i < 2 % going to bowl
        times(i+1) = times(i) + 0.5;
    elseif i == 2 % scooping down
        times(i+1) = times(i) + 1;
    elseif i == 4 % scooping up
        times(i+1) = times(i) + 1.5;
    elseif i == 5 % moving from bowl to mouth (SLOW)
        times(i+1) = times(i) + 2;
    elseif i == 6 % moving forward
        times(i+1) = times(i) + 1;
    else % dropping bullets into mouth and return
        times(i+1) = times(i) + 0.5;
    end 
end

trajectory = trajectory_const_vel(waypoints(:,1:2), times(1:2), frequency/2);
stop = command_trajectory(robotHardware, trajectory, frequency, robot, 0);

stop = 0;
while (~stop)
    % --------------- BEGIN STUDENT SECTION ----------------------------------    
    trajectory = trajectory_const_vel(waypoints(:,2:4), times(2:4)-ones(1,3)*times(2), frequency/2);
    stop = command_trajectory(robotHardware, trajectory, frequency, robot, 0);
    if ~stop
        trajectory = trajectory_spline(waypoints(:,4:6), times(4:6)-ones(1,3)*times(4), frequency);
        stop = command_trajectory(robotHardware, trajectory, frequency, robot, 1);
    end
    if ~stop
        trajectory = trajectory_const_vel(waypoints(:,6:10), times(6:10)-ones(1,5)*times(6), frequency/2);
        stop = command_trajectory(robotHardware, trajectory, frequency, robot, 1);
    end       
    % --------------- END STUDENT SECTION ------------------------------------
end

% Return to home position
homePositioning(robotHardware, frequency, robot);

%% Stop logging, and plot results
robotHardware.stopLog();

hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));

% Plot Position Error
figure();
for i = 1:5
    subplot(5,1,i);
    expected = hebilog.positionCmd(:,i);
    actual = hebilog.position(:,i);
    error = abs(expected-actual);
    time = hebilog.time;
    plot(time, error);
    name = sprintf('Joint %d Position Error', i);
    title(name);
end

% Plot Torque
figure();
for i = 1:5
    subplot(5,1,i);
    torque = hebilog.torque(:,i);
    time = hebilog.time;
    plot(time, torque);
    name = sprintf('Joint %d Torque', i);
    title(name);
end

% Plot angle data
figure();
subplot(3,1,1);
plot(hebilog.time, hebilog.positionCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.position, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta');
subplot(3,1,2);
plot(hebilog.time, hebilog.velocityCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.velocity, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint velocities during trajectory');
xlabel('t');
ylabel('joint velocities');
subplot(3,1,3);
plot(hebilog.time, hebilog.torque, 'r--', 'LineWidth', 1)
title('Plot of joint torques during trajectory');
xlabel('t');
ylabel('\tau');

end