function [ trajectory ] = trajectory_const_vel(waypoints, times, frequency)
% trajectory_const_vel
%
%   Returns a matrix of joint angles, where each column represents a single
%   timestamp. These joint angles form constant velocity trajectory segments,
%   hitting waypoints(:,i) at times(i).
%
%   'waypoints' is a matrix of waypoints; each column represents a single
%   waypoint in joint space, and each row represents a particular joint.
%
%   'times' is a row vector that indicates the time each of the waypoints should
%   be reached.  The number of columns should equal the number of waypoints, and
%   should be monotonically increasing.  The first number is technically
%   arbitrary, as it serves only as a reference for the remaining timestamps,
%   but '0' is a good default.
%
%   'frequency' is the control frequency which this trajectory should be played
%   at, and therefore the number of columns per second of playback.

% Number of joints:
num_joints = size(waypoints, 1);
% Number of waypoints:
num_waypoints = size(waypoints, 2);
% Number of segments between waypoints:
num_segments = num_waypoints - 1;

if (size(times) ~= [1, num_waypoints])
    error('Size of times vector is incorrect!');
end

if (num_waypoints < 2)
    error('Insufficient number of waypoints.');
end

if (length(frequency) ~= 1 || frequency < 5)
    error('Invalid control frequency (must be at least 5Hz)');
end

%% First, we go through each segment between waypoints, and calculate how many
%% points we must generate in this segment given our control frequency.
num_points_per_segment = zeros(1, num_segments);
for segment = 1:num_segments
    dt = times(segment+1) - times(segment);
    num_points_per_segment(segment) = dt * frequency;
end

%% Go through each segment of the trajectory, and fill it in segment-by-segment.

% Pre-allocate a trajectory matrix with the correct number of rows and columns.
trajectory = zeros(num_joints, sum(num_points_per_segment));

% The index of the first point in this segment
segment_start_point = 1;
for segment = 1:num_segments
    % Number of points in this segment
    points_in_segment = num_points_per_segment(segment);

    % The index of the last point in this segment
    segment_end_point = segment_start_point + points_in_segment - 1;

    % --------------- BEGIN STUDENT SECTION ----------------------------------
    % Fill in the points in this segment of the trajectory.  Those points will
    % be:
    %   trajectory(:, segment_start_point:segment_end_point)
    % HINT: you can use your code form last week's homework to help!
    
    theta1 = waypoints(:,segment);
    theta2 = waypoints(:,segment+1);
    traj_seg = zeros(num_joints,points_in_segment);
    
    for i = 1:num_joints
        traj_seg(i,:) = linspace(theta1(i), theta2(i), points_in_segment);
    end
    
    trajectory(:, segment_start_point:segment_end_point) = traj_seg;
    
    % --------------- END STUDENT SECTION ------------------------------------

    % Update the starting index as we move to the next segment.
    segment_start_point = segment_start_point + points_in_segment;
end

end
