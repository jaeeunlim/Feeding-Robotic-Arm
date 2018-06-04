% Define a simple function to actually send a series of points to the
% robot, where the 'trajectory' is a matrix of columns of joint angle
% commands to be sent to 'robot' at approximately 'frequency'.
% Note this also commands velocities, although you can choose to only command
% positions if desired, or to add torques to help compensate for gravity.
function [stop] = command_trajectory(robotHardware, trajectory, frequency, robot, checkstop)
  %% Setup reusable structures to reduce memory use in loop
  cmd = CommandStruct();
  
  % Compute the velocity numerically
  trajectory_vel = diff(trajectory, 1, 2);

  % Command the trajectory
  stop = 0;
  for i = 1:(size(trajectory, 2) - 1)
    threshold = 5;
    fbk = robotHardware.getNextFeedback();
    t = fbk.torque';
    J = robot.jacobian(fbk.position');
    f = J*t;
    F = sqrt(f(1)^2+f(2)^2+f(3)^2);
    
    if checkstop & F > threshold
        stop = 1;
        break;
    end
      
    % Send command to the robot (the transposes on the trajectory
    % points turns column into row vector for commands).
    cmd.position = trajectory(:,i)';
%     cmd.velocity = trajectory_vel(:,i)' * frequency;
    robotHardware.set(cmd);

    % Wait a little bit to send at ~100Hz.
    pause(1 / frequency);
  end

  % Send the last point, with a goal of zero velocity.
  cmd.position = trajectory(:,end)';
  cmd.velocity = zeros(1, size(trajectory, 1));
  robotHardware.set(cmd);
end