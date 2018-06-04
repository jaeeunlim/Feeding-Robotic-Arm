classdef Robot3D
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        link_lengths
        link_twists
        link_offsets
        joint_angles
%         link_masses
%         joint_masses
%         end_effector_mass
    end
    
    methods
        % Constructor: Makes a brand new robot with the specified parameters.
        function robot = Robot3D(DH_parameters) 
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(DH_parameters, 2) ~= 4
               error('Invalid DH matrix: Should have four parameters, is %dx%d.', size(DH_parameters, 1), size(DH_parameters, 2));
            end
            
            robot.dof = size(DH_parameters, 1);
            
            robot.link_lengths = DH_parameters(:,1);
            robot.link_twists = DH_parameters(:,2);
            robot.link_offsets = DH_parameters(:,3);
            robot.joint_angles = DH_parameters(:,4);
%             robot.link_masses = link_masses;
%             robot.joint_masses = joint_masses;
%             robot.end_effector_mass = end_effector_mass;  
        end
       
        % Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        function frames = forward_kinematics(robot,thetas)
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end
            
            % Allocate a variable containing the transforms from each frame
            % to the base frame.
            n = robot.dof;
            frames = zeros(4,4,n); %%%%%%%%%%%%
            % The transform from the base of link 'i' to the base frame (H^0_i)
            % is given by the 4x4 matrix frames(:,:,i).

            % The transform from the end effector to the base frame (H^0_i) is
            % given by the 4x4 matrix frames(:,:,end).

            
            %% FILL IN 4x4 HOMOGENEOUS TRANSFORM FOR n + 1 FRAMES
            alphas = robot.link_twists;
            links = robot.link_lengths;
            offsets = robot.link_offsets;
            thetas = thetas + robot.joint_angles;
            
            % First frame 
            a = alphas(1);
            th = thetas(1);
            l = links(1);
            d = offsets(1);

            cth = cos(th);
            sth = sin(th);
            ca = cos(a);
            sa = sin(a);
            
            frames(:,:,1) = [cth, -sth*ca,  sth*sa, l*cth;
                             sth,  cth*ca, -cth*sa, l*sth;
                               0,      sa,      ca,     d;
                               0,       0,       0,     1];
           
            for i = 2:n
                % Get the DH paramters
                a = alphas(i);
                th = thetas(i);
                l = links(i);
                d = offsets(i);

                cth = cos(th);
                sth = sin(th);
                ca = cos(a);
                sa = sin(a);
                
                A = [cth, -sth*ca,  sth*sa, l*cth;
                     sth,  cth*ca, -cth*sa, l*sth;
                       0,      sa,      ca,     d;
                       0,       0,       0,     1];
                H_0_i_1 = frames(:,:,i-1);
                frames(:,:,i) = H_0_i_1*A; 
            end         
        end
       
        % Shorthand for returning the forward kinematics.
        function fk = fk(robot,thetas)
            fk = robot.forward_kinematics(thetas);
        end
       
        % Returns [x; y; z; roll; pitch; yaw] for the end effector given a set of joint
        % angles. 
        function ee = end_effector(robot,thetas)
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);
            H_0_ee = frames(:,:,end);
           
            % position
            x = H_0_ee(1,4);
            y = H_0_ee(2,4);
            z = H_0_ee(3,4);
            
            % orientation
            pitch = asin(-H_0_ee(3,1));
            r11 = H_0_ee(1,1);
            r12 = H_0_ee(2,1);
            roll = atan2(r12,r11);
            r32 = H_0_ee(3,2);
            r33 = H_0_ee(3,3);
            yaw = atan2(r32,r33);
           
            % end-effector
            ee = [x; y; z; roll; pitch; yaw];
        end
       
        % Shorthand for returning the end effector position and orientation. 
        function ee = ee(robot,thetas)
            ee = robot.end_effector(thetas);
        end
        
        % Jacobian
        function jacobian = jacobian(robot,thetas)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end

            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            n = robot.dof;
            jacobian = zeros(6,n); %%%%%%%%%%%%%%% 

% --------------- BEGIN STUDENT SECTION ----------------------------------
            base_frames = robot.fk(thetas);
            ee = robot.ee(thetas);
            z = [0;0;1];
            o = ee(1:3);
            jacobian(:,1) = vertcat(cross(z,o),z); 
            for i = 2:n
                frame = base_frames(:,:,i-1);
                z = frame(1:3,3);
                o = ee(1:3) - frame(1:3,4);
                jacobian(:,i) = vertcat(cross(z,o),z); 
            end
% --------------- END STUDENT SECTION ------------------------------------
        end
            
                   
        % Inverse Kinematics
        function thetas = inverse_kinematics(robot, initial_thetas, goal_position)
            
            function e = error_fun(x, robot, goal_position)
                goal = goal_position;
                endeff = robot.ee(x);
                e = sqrt(sum((goal(1:3)-endeff(1:3)).^2))*100 + sqrt(sum((goal(4:6)-endeff(4:6)).^2));
            end
            
            n = size(initial_thetas);
            LB = [    -0.1;   0; 0; -pi/2; -pi];
            UB = [pi/2+0.1; pi+0.1;  pi;  pi/2;  pi];
            thetas = fmincon(@(x) error_fun(x,robot,goal_position),...
                            initial_thetas,[],[],[],[],LB,UB);
        end
        
        function ik = ik(robot,initial_thetas,goal_position)
            ik = robot.inverse_kinematics(initial_thetas,goal_position);
        end
    end
end
