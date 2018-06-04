% 16-384 Robot Kinematics and Dynamics
% Capstone 2017

close all;
clear all;
clc;

%% Question 1

% Robot A
th1 = 45;
d2 = 2;
d3 = 3;

link_lengths = [0;0;0];
link_twists = [0;pi/2;0];
link_offsets = [1;d2;d3];
joint_angles = [0;pi/2;0];
thetas = [th1;0;0]*pi/180;

% Robot B
d1 = 2;
d2 = 3;
d3 = 1;
th4 = 20;
th5 = 45;
th6 = 90;

link_lengths = [0;0;0;0;0;0];
link_twists = [-pi/2;-pi/2;pi;-pi/2;pi/2;pi];
link_offsets = [1+d1;1+d2;d3;0;0;0];
joint_angles = [-pi/2;pi/2;0;0;0;0];
thetas = [0;0;0;th4;th5;th6]*pi/180;

% Robot C
th1 = 20;
th2 = 45;
th3 = 45;
th4 = -45;
th5 = 90;

link_lengths = [0;0.38;0.36;0;0];
link_twists = [pi/2;pi;pi;pi/2;0];
link_offsets = [0;0.09;0.07;0.09;0.115];
joint_angles = [0;0;0;pi/2;0];
thetas = [th1;th2;th3;th4;th5]*pi/180;

% General
% DH parameters
n = length(thetas);
DH_parameters = zeros(n,4);
DH_parameters(:,1) = link_lengths;
DH_parameters(:,2) = link_twists;
DH_parameters(:,3) = link_offsets;
DH_parameters(:,4) = joint_angles;

% robot instance
robot = Robot3D(DH_parameters);
transformation = robot.fk(thetas)


%% Question 2&3: Workspace & Manipulability

th1 = 0;
th2s = linspace(-180,180,100);
th3s = linspace(-180,180,100);
th4 = 0;
th5 = 0;

x = [];
z = [];
u = [];
for i = 1:length(th2s)
    th2 = th2s(i);
    for j = 1:length(th3s)
        th3 = th3s(j);
        th4 = th3-th2-pi/2;
        
        link_lengths = [0;0.38;0.36;0;0];
        link_twists = [pi/2;pi;pi;pi/2;0];
        link_offsets = [0;0.09;0.07;0.09;0.115];
        joint_angles = [0;0;0;pi/2;0];
        thetas = [th1;th2;th3;th4;th5]*pi/180;
        thetas2 = [th2;th3]*pi/180;

        % DH parameters
        n = length(thetas);
        DH_parameters = zeros(n,4);
        DH_parameters(:,1) = link_lengths;
        DH_parameters(:,2) = link_twists;
        DH_parameters(:,3) = link_offsets;
        DH_parameters(:,4) = joint_angles;

        % robot workspace
        robot = Robot3D(DH_parameters);
        endeff = robot.ee(thetas);
        x(end+1) = endeff(1);
        z(end+1) = endeff(3);
        
        % manipulability
        link_length = [.38; .36];
        robot2 = Robot(link_length,[1;1],[1;1],1);
        J = robot2.jacobian2(thetas2);
        J = J(:,:,end);
        J = J(1:2,:);   
        u(end+1) = sqrt(det(J*J'));
    end
end

% plot workspace
figure();
plot(z,x,'o');
xlabel('x (m)');
ylabel('z (m)');
axis equal;
title('End-Effector Workspace');

% plot manipulability
figure();
pointsize = 50;
scatter3(z,x,u,pointsize,u,'filled');
colorbar;
xlabel('x (m)');
ylabel('z (m)');
zlabel('u');
axis equal;
title('Yoshikawa Manipulability');


%% Question 5: 3D Jacobian

link_lengths = [0;0.38;0.36;0;0];
link_twists = [pi/2;pi;pi;pi/2;0];
link_offsets = [0;0.09;0.07;0.09;0.115];
joint_angles = [0;0;0;0;0];
thetas = [0;pi/2;pi/4;pi/4;0];

% DH parameters
n = length(thetas);
DH_parameters = sym(zeros(n,4));
DH_parameters(:,1) = link_lengths;
DH_parameters(:,2) = link_twists;
DH_parameters(:,3) = link_offsets;
DH_parameters(:,4) = joint_angles;

% robot instance
robot = Robot3D(DH_parameters);
J = robot.jacobian(thetas)


%% Test IK
th1 = 0;
th2 = 0;
th3 = 0;
th4 = 0;
th5 = 0;

link_lengths = [0;0.38;0.36;0;0];
link_twists = [pi/2;pi;pi;pi/2;0];
link_offsets = [0;0.09;0.07;0.09;0.115];
joint_angles = [0;0;0;pi/2;0];
thetas = [th1;th2;th3;th4;th5]*pi/180;

% DH parameters
n = length(thetas);
DH_parameters = zeros(n,4);
DH_parameters(:,1) = link_lengths;
DH_parameters(:,2) = link_twists;
DH_parameters(:,3) = link_offsets;
DH_parameters(:,4) = joint_angles;

% robot instance
initial_thetas = [0;0;0;0;0];
goal_position = [2;2;2;2;2;2];
robot = Robot3D(DH_parameters);
ik = robot.ik(initial_thetas,goal_position)
