clear; close; clc;
% ROS Setup
rosinit;
j1_pos = rospublisher('/fridge_bot/joint1_position_controller/command');
j3_pos = rospublisher('/fridge_bot/joint3_position_controller/command');
j7_pos = rospublisher('/fridge_bot/joint7_position_controller/command');

JointStates = rossubscriber('/fridge_bot/joint_states');
q1 = rosmessage(j1_pos);
q3 = rosmessage(j3_pos);
q7 = rosmessage(j7_pos);
% tau1.Data = 0;
% tau3.Data = -10;
% tau7.Data = -10;
% send(j1_effort,tau1);
% send(j3_effort,tau3);
% send(j7_effort,tau7);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'fridge_bot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint3','joint7'};
req.JointPositions = [deg2rad(0), deg2rad(0),deg2rad(0)];
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
% syms t;
% traj1 = (pi*(- (3*t^3)/25 + (9*t^2)/5))/180;
% traj2 = (pi*(- (9*t^3)/50 + (27*t^2)/10))/180;
% traj3 = (pi*(- (9*t^3)/50 + (27*t^2)/10))/180;
% 
% traj1dot = diff(traj1,t);
% traj2dot = diff(traj2,t);
% traj3dot = diff(traj3,t);
% 
% traj1ddot = diff(traj1dot,t);
% traj2ddot = diff(traj2dot,t);
% traj3ddot = diff(traj3dot,t);

% trajddot = [traj1ddot;traj2ddot;traj3ddot];
while(t1 < 10)
t1 = toc;
% read the joint states
jointData = receive(JointStates);
% inspect the "jointData" variable in MATLAB to get familiar with its structure
Z = [jointData.Position(1);jointData.Position(2);jointData.Position(3);jointData.Velocity(1);jointData.Velocity(2);jointData.Velocity(3)];
Efforts = [jointData.Effort(1);jointData.Effort(2);jointData.Effort(3)];
% q1 = Z(1);
% q2 = Z(2);
% q3 = Z(3);
% q_dot_1 = Z(4);
% q_dot_2 = Z(5);
% q_dot_3 = Z(6);


% K = [12.0000         0         0    7.0000         0         0;
%          0    2.0000         0         0    3.0000         0;
%          0         0    2.0000         0         0    3.0000];
% 
% I1 = 0; I2 = 0; I3 = 0; l1R = 31.85; l1D = 25; l2R = 200; l2D = 0; l3R = 200; l3D = 0;
% lc1R = -6.37; lc1D = 14.26; lc2R = 110.86; lc2D = 4.06; lc3R = 122.2; lc3D = 31; 
% m1 = 0.1923; m2 = 0.08557; m3 = 0.09533;

% U = (-1*K*Z);
% u1 = U(1)
% u3 = U(2)
% u7 = U(3)
% 
% M = [I1 + I2 + I3 + l2D^2*m3 + l1R^2*m2 + l1R^2*m3 + (l2R^2*m3)/2 + lc2D^2*m2 + lc3D^2*m3 + lc1R^2*m1 + (lc2R^2*m2)/2 + (lc3R^2*m3)/2 + 2*l2D*lc3D*m3 + (l2R^2*m3*cos(2*q2))/2 + (lc2R^2*m2*cos(2*q2))/2 + (lc3R^2*m3*cos(2*q2 + 2*q3))/2 + 2*l1R*lc3R*m3*cos(q2 + q3) + 2*l1R*l2R*m3*cos(q2) + 2*l1R*lc2R*m2*cos(q2) + l2R*lc3R*m3*cos(q3) + l2R*lc3R*m3*cos(2*q2 + q3), - l2D*lc3R*m3*sin(q2 + q3) - lc3D*lc3R*m3*sin(q2 + q3) - l2D*l2R*m3*sin(q2) - l2R*lc3D*m3*sin(q2) - lc2D*lc2R*m2*sin(q2),   -lc3R*m3*sin(q2 + q3)*(l2D + lc3D);
%     - l2D*lc3R*m3*sin(q2 + q3) - lc3D*lc3R*m3*sin(q2 + q3) - l2D*l2R*m3*sin(q2) - l2R*lc3D*m3*sin(q2) - lc2D*lc2R*m2*sin(q2),                                                       m3*l2R^2 + 2*m3*cos(q3)*l2R*lc3R + m2*lc2R^2 + m3*lc3R^2 + I2 + I3, m3*lc3R^2 + l2R*m3*cos(q3)*lc3R + I3;
%     -lc3R*m3*sin(q2 + q3)*(l2D + lc3D),                                                                                     m3*lc3R^2 + l2R*m3*cos(q3)*lc3R + I3,                       m3*lc3R^2 + I3];
% 
% Gq = [ 0;
% m3*(lc3R*cos(q2 + q3) + l2R*cos(q2)) + lc2R*m2*cos(q2);
%                                   lc3R*m3*cos(q2 + q3)];  
% 
% Cq = [- q_dot_2*((m3*sin(2*q2)*l2R^2)/2 + m3*sin(2*q2 + q3)*l2R*lc3R + l1R*m3*sin(q2)*l2R + (m2*sin(2*q2)*lc2R^2)/2 + l1R*m2*sin(q2)*lc2R + (m3*sin(2*q2 + 2*q3)*lc3R^2)/2 + l1R*m3*sin(q2 + q3)*lc3R) - (lc3R*m3*q_dot_3*(lc3R*sin(2*q2 + 2*q3) + 2*l1R*sin(q2 + q3) + l2R*sin(q3) + l2R*sin(2*q2 + q3)))/2, q_dot_1*((m3*sin(2*q2)*l2R^2)/2 + m3*sin(2*q2 + q3)*l2R*lc3R + l1R*m3*sin(q2)*l2R + (m2*sin(2*q2)*lc2R^2)/2 + l1R*m2*sin(q2)*lc2R + (m3*sin(2*q2 + 2*q3)*lc3R^2)/2 + l1R*m3*sin(q2 + q3)*lc3R), (lc3R*m3*q_dot_1*(lc3R*sin(2*q2 + 2*q3) + 2*l1R*sin(q2 + q3) + l2R*sin(q3) + l2R*sin(2*q2 + q3)))/2;
% - q_dot_1*((m3*sin(2*q2)*l2R^2)/2 + m3*sin(2*q2 + q3)*l2R*lc3R + l1R*m3*sin(q2)*l2R + (m2*sin(2*q2)*lc2R^2)/2 + l1R*m2*sin(q2)*lc2R + (m3*sin(2*q2 + 2*q3)*lc3R^2)/2 + l1R*m3*sin(q2 + q3)*lc3R) - q_dot_2*(l2D*lc3R*m3*cos(q2 + q3) + lc3D*lc3R*m3*cos(q2 + q3) + l2D*l2R*m3*cos(q2) + l2R*lc3D*m3*cos(q2) + lc2D*lc2R*m2*cos(q2)) - lc3R*m3*q_dot_3*cos(q2 + q3)*(l2D + lc3D), -l2R*lc3R*m3*q_dot_3*sin(q3),  l2R*lc3R*m3*q_dot_2*sin(q3);
%                                                                                                                                                                                   - (lc3R*m3*q_dot_1*(lc3R*sin(2*q2 + 2*q3) + 2*l1R*sin(q2 + q3) + l2R*sin(q3) + l2R*sin(2*q2 + q3)))/2 - lc3R*m3*q_dot_2*cos(q2 + q3)*(l2D + lc3D) - lc3R*m3*q_dot_3*cos(q2 + q3)*(l2D + lc3D),   -l2R*lc3R*m3*sin(q3)*(q_dot_2 + q_dot_3),  0]
% traj1 = subs(traj1,t,t1); 
% traj2 = subs(traj2,t,t1); 
% traj3 = subs(traj3,t,t1);
% traj1dot = subs(traj1dot,t,t1); traj2dot = subs(traj2dot,t,t1);  traj3dot = subs(traj3dot,t,t1); trajddot = subs(trajddot,t,t1);
% error = [Z(1)-traj1;Z(2)-traj2;Z(3)-traj3; Z(4)-traj1dot;Z(5)-traj2dot;Z(6)-traj3dot];
% t1
% v = (-1*K*error)+trajddot;
% 
% 
% 
% Tau = M*v + Cq +Gq;
% 
% u1 = double(Tau(1))
% u3 = double(Tau(2))
% u7 = double(Tau(3))

torq1 = [torq1;Efforts(1)];
torq2 = [torq2;Efforts(1)];
torq3 = [torq3;Efforts(1)];
T = [T;toc];
z1 = [z1;Z(1)];
z2 = [z2;Z(2)];
z3 = [z3;Z(3)];
z4 = [z4;Z(4)];
z5 = [z5;Z(5)];
z6 = [z6;Z(6)];
% design your state feedback controller in the following
q1.Data = deg2rad(120);
q3.Data = deg2rad(10);
q7.Data = -deg2rad(40);
send(j1_pos,q1);
send(j3_pos,q3);
send(j7_pos,q7);
% you can sample data here to be plotted at the end
end
% tau1.Data = 0;
% tau3.Data = 0;
% tau7.Data = 0;
% send(j1_effort,tau1);
% send(j3_effort,tau3);
% send(j7_effort,tau7);
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