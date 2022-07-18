clear; close; clc;
% ROS Setup
rosinit;
j1_effort = rospublisher('/fridge_bot/joint1_effort_controller/command');
j3_effort = rospublisher('/fridge_bot/joint3_effort_controller/command');
j7_effort = rospublisher('/fridge_bot/joint7_effort_controller/command');

JointStates = rossubscriber('/fridge_bot/joint_states');
tau1 = rosmessage(j1_effort);
tau3 = rosmessage(j3_effort);
tau7 = rosmessage(j7_effort);
tau1.Data = 0;
tau3.Data = -10;
tau7.Data = -10;
send(j1_effort,tau1);
send(j3_effort,tau3);
send(j7_effort,tau7);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'fridge_bot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint3','joint7'};
req.JointPositions = [deg2rad(30), deg2rad(90),deg2rad(0)];
resp = call(client,req,'Timeout',3);
tic;
t1 = 0;

z1 = [];
z2 = [];
z3 = [];
z4 = [];
z5 = [];
z6 = [];
torq1 = [];
torq2 = [];
torq3 = [];
T = [];
syms t;
traj1 = (pi*(- (3*t^3)/25 + (9*t^2)/5))/180;
traj2 = (pi*(- (9*t^3)/50 + (27*t^2)/10))/180;
traj3 = (pi*(- (9*t^3)/50 + (27*t^2)/10))/180;

traj1dot = diff(traj1,t);
traj2dot = diff(traj2,t);
traj3dot = diff(traj3,t);

traj1ddot = diff(traj1dot,t);
traj2ddot = diff(traj2dot,t);
traj3ddot = diff(traj3dot,t);

trajddot = [traj1ddot;traj2ddot;traj3ddot];
while(t1 < 10)
t1 = toc;
% read the joint states
jointData = receive(JointStates);
% inspect the "jointData" variable in MATLAB to get familiar with its structure
Z = [jointData.Position(1)-deg2rad(90);jointData.Position(2)-deg2rad(45);jointData.Position(3)+deg2rad(30);jointData.Velocity(1);jointData.Velocity(2);jointData.Velocity(3)];

q1 = Z(1);
q2 = Z(2);
q3 = Z(3);
q_dot_1 = Z(4);
q_dot_2 = Z(5);
q_dot_3 = Z(6);


K = [12.0000         0         0    7.0000         0         0;
         0    2.0000         0         0    3.0000         0;
         0         0    2.0000         0         0    3.0000];

I1 = 0; I2 = 0; I3 = 0; l1R = 31.85; l1D = 25; l2R = 200; l2D = 0; l3R = 200; l3D = 0;
lc1R = -6.37; lc1D = 14.26; lc2R = 110.86; lc2D = 4.06; lc3R = 122.2; lc3D = 31; 
m1 = 0.1923; m2 = 0.08557; m3 = 0.09533;

% Control Law
U = (-1*K*Z);
u1 = U(1)
u3 = U(2)
u7 = U(3)

torq1 = [torq1;u1];
torq2 = [torq2;u3];
torq3 = [torq3;u7];
T = [T;toc];
z1 = [z1;Z(1)];
z2 = [z2;Z(2)];
z3 = [z3;Z(3)];
z4 = [z4;Z(4)];
z5 = [z5;Z(5)];
z6 = [z6;Z(6)];
% design your state feedback controller in the following
tau1.Data = u1;
tau3.Data = u3;
tau7.Data = u7;
send(j1_effort,tau1);
send(j3_effort,tau3);
send(j7_effort,tau7);
% you can sample data here to be plotted at the end
end
tau1.Data = 0;
tau3.Data = 0;
tau7.Data = 0;
send(j1_effort,tau1);
send(j3_effort,tau3);
send(j7_effort,tau7);
% disconnect from roscore
rosshutdown;

subplot(3,3,1);
plot(T,z1)
xlabel("time")
ylabel("radians")
title("theta1")


subplot(3,3,2);
plot(T,z2)
xlabel("time")
ylabel("radians")
title("theta2")

subplot(3,3,3);
plot(T,z3)
xlabel("time")
ylabel("rad/s")
title("theta3")

subplot(3,3,4);
plot(T,z4)
xlabel("time")
ylabel("rad/s")
title("theta1_dot")

subplot(3,3,5);
plot(T,z5)
xlabel("time")
ylabel("rad/s")
title("theta2_dot")

subplot(3,3,6);
plot(T,z6)
xlabel("time")
ylabel("rad/s")
title("theta3_dot")

subplot(3,3,7);
plot(T,torq1)
xlabel("time")
ylabel("N/m")
title("u1")

subplot(3,3,8);
plot(T,torq2)
xlabel("time")
ylabel("N/m")
title("u2")

subplot(3,3,9);
plot(T,torq3)
xlabel("time")
ylabel("N/m")
title("u3")

% figure;