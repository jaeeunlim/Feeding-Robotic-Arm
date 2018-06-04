function [ trajectory ] = trajectory_spline(waypoints, times, frequency)
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

% --------------- BEGIN STUDENT SECTION ----------------------------------

% Compute the cubic spline which interpolates the given waypoints, and has zero
% velocity at each endpoint.
%
% A spline can be directly filled in all at once, by using the 'spline' function.
%
% Example: given 't' and 'y' (values of unknown function at 't'):
%   yy = spline(x, y, xx) 
% returns interpolated values 'yy' at each point 'xx'.
%
% Note: spline(x, [0 y 0], xx) constrains the first/last point to have% zero
% velocity. Here, '0' must be sized appropriately for the number of rows in 'y'.

trajectory = zeros(num_joints, sum(num_points_per_segment)); % Replace this with actual value of trajectory!

n = sum(num_points_per_segment);
interpolated_times = linspace(times(1),times(end),n);

trajectory = spline(times, [zeros(num_joints,1) waypoints zeros(num_joints,1)], interpolated_times);

% --------------- END STUDENT SECTION ------------------------------------

end
