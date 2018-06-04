% homePositioning(robot, robotHardware, frequency)
%   inputs:
%       robot: object of robot class (required to do inverse kinematics)
%       robotHardware: hebi group 
%       frequency: rate at which you want to send commands to robot
%   output: 
%       trajectory: the final trajectory from initial positions to homing
%       position
% The function takes the robot from any initial position to home position
% in two steps. First step is to move the shoulder joint to safe position.
% This is done to safe position the other joints so that they do not hit
% the table while you move them to home position.

function [trajectory] = homePositioning(robotHardware, frequency, robot)
    homePosition = [0; pi/2; pi/2; 0.15; 0];
    %Get initial position
    fbk = robotHardware.getNextFeedback();
    initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)
    % Move shoulder to safe location
    homingTime = 2;
    firstSafeTrajectory = repmat(initial_thetas, 1, homingTime*frequency);
    firstSafeTrajectory(2,:) = linspace(initial_thetas(2), 2*homePosition(2), homingTime*frequency);
    command_trajectory(robotHardware, firstSafeTrajectory, frequency, robot, 0);

    % Move remaining joints to safe loacation
    % Get initial position
    fbk = robotHardware.getNextFeedback();
    initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)
    trajectory = zeros(size(firstSafeTrajectory));
    for i = 1:size(homePosition,1)
        trajectory(i,:) = linspace(initial_thetas(i), homePosition(i), homingTime*frequency);
    end
    command_trajectory(robotHardware, trajectory, frequency, robot, 0);
end