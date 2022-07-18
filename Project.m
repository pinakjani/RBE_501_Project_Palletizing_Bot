clc;
clear;
syms m1 m2 m3 real
syms l1R l1D l2R l2D l3R l3D lc1R lc1D lc2R lc2D lc3R lc3D real
syms q1 q2 q3 real
syms I1 I2 I3 real
syms g real
syms q_dot_1 q_dot_2 q_dot_3 q_ddot_1 q_ddot_2 q_ddot_3 real
syms th1(t) th2(t) th3(t)

% theta, D, R, alpha
DH= [q1 lc1D lc1R 0;
    q1 l1D l1R pi/2;
    q2 lc2D lc2R 0;
    q2 l2D l2R 0;
    q3 lc3D lc3R 0;
    q3 l3D l3R 0];

for c = 1:6
    Transforms(:,:,c) = dhparam2matrix(DH(c,1),DH(c,2),DH(c,3),DH(c,4));
end

T03 = Transforms(:,:,2)*Transforms(:,:,4)*Transforms(:,:,6);

mass1_T = Transforms(:,:,1);
mass1_pos = mass1_T(1:3,4);
Jv1 = [diff(mass1_pos, q1) [0;0;0] [0;0;0]];
Jw1 = [[0;0;1] [0;0;0] [0;0;0]];
R1 = mass1_T(1:3, 1:3);

mass2_T = Transforms(:,:,2)*Transforms(:,:,3);
mass2_pos = mass2_T(1:3,4);
Jv2 = [diff(mass2_pos, q1) diff(mass2_pos, q2) [0;0;0]];
Jw2 = [[0;0;1] [0;-1;0] [0;0;0]];
R2 = mass2_T(1:3, 1:3);

mass3_T = Transforms(:,:,2)*Transforms(:,:,4)*Transforms(:,:,5);
mass3_pos = mass3_T(1:3,4);
Jv3 = [diff(mass3_pos, q1) diff(mass3_pos, q2) diff(mass3_pos, q3)];
Jw3 = [[0;0;1] [0;-1;0] [0;-1;0]];
R3 = mass3_T(1:3, 1:3);

Kw1 = Jw1.' * R1*I1*R1.'*Jw1;
Kw2 = Jw2.' * R2*I2*R2.'*Jw2;
Kw3 = Jw3.' * R3*I3*R3.'*Jw3;

K1 = m1*(Jv1.')*Jv1 + Kw1;
K2 = m2*(Jv2.')*Jv2 + Kw2;
K3 = m3*(Jv3.')*Jv3 + Kw3;

% D 3x3 matrix question 1
D = simplify(K1 + K2 + K3)

K = simplify((1/2)*[q_dot_1 q_dot_2 q_dot_3]*D*[q_dot_1; q_dot_2; q_dot_3]);
theta = [q1, q2, q3];
for i = 1:3
    for j = 1:3
        for k = 1:3
            C(i,j,k) = 1/2 * ( ...
            diff(D(k,j), theta(i)) + ...
            diff(D(k,i), theta(j)) - ...
            diff(D(i,j), theta(k)));
        end
    end
end
C = simplify(C); 
C_kj = sym(zeros(3,3));
theta_dot = [q_dot_1, q_dot_2, q_dot_3];
for i = 1:3
    for j = 1:3
        for k = 1:3
            C_kj(j,k) = C_kj(j,k) + (C(i,j,k) * theta_dot(i));
        end
    end
end
% Christoffel Symbols question 2
C_kj = simplify(C_kj)
% Potential Energy question 3
P = mass1_pos(3) * m1 + mass2_pos(3) * m2 + mass3_pos(3) * m3;
for i = 1:3
    g(i) = diff(P, theta(i));
end
% Gravity Term question 3
g = simplify(g.')
tau = (D * [q_ddot_1;q_ddot_2;q_ddot_3]) + (C_kj * [q_dot_1;q_dot_2;q_dot_3])+g;
% Dynamical Model Question 4
tau = simplify(tau)
% World Orgion 0,0,0
% Robot Origin 66.05,24.54,-34.8
% Link 1 CoM 71.76,38.80,-28.43
%   D = 14.26 R = -6.37
% Link 2 CoM 70.62, 160.40, -7.01
%   D = 4.06, R = 110.86
% Link 3 CoM -42.08, 264.53, -34.79
%   D = 31, R = 122.2
% Link 1->2 D = 25 R = 31.85
% Link 2->3 D = 0, R = 200
% Link 2->3 D = 0, R = 200

tau_subs = subs(tau,[l1R l1D l2R l2D l3R l3D lc1R lc1D lc2R lc2D lc3R lc3D],[31.85,25,200,0,200,0,-6.37,14.26,110.86,4.06,122.2,31]);
simplify(tau_subs)

syms a0 a1 a2 a3 b0 b1 b2 b3 c0 c1 c2 c3;

trajeqn1 = [a0 + a1*t + a2*t^2 + a3*t^3 - q1;
        a1 + 2*a2*t + 3*a3*t^2 - q_dot_1];
trajeqn2 = [b0 + b1*t + b2*t^2 + b3*t^3 - q2;
        b1 + 2*b2*t + 3*b3*t^2 - q_dot_2];
trajeqn3 = [c0 + c1*t + c2*t^2 + c3*t^3 - q3;
        c1 + 2*c2*t + 3*c3*t^2 - q_dot_3];

sol = solve([subs(trajeqn1,[t,q1,q_dot_1],[0,0,0]);subs(trajeqn1,[t,q1,q_dot_1],[10,60,0])],[a0,a1,a2,a3]);

trajeqn1 = subs(trajeqn1,[a0,a1,a2,a3],[sol.a0,sol.a1,sol.a2,sol.a3]);

sol = solve([subs(trajeqn2,[t,q2,q_dot_2],[0,0,0]);subs(trajeqn2,[t,q2,q_dot_2],[10,90,0])],[b0,b1,b2,b3]);

trajeqn2 = subs(trajeqn2,[b0,b1,b2,b3],[sol.b0,sol.b1,sol.b2,sol.b3]);

sol = solve([subs(trajeqn3,[t,q3,q_dot_3],[0,0,0]);subs(trajeqn3,[t,q3,q_dot_3],[10,90,0])],[c0,c1,c2,c3]);

trajeqn3 = subs(trajeqn3,[c0,c1,c2,c3],[sol.c0,sol.c1,sol.c2,sol.c3]);

traj1 = deg2rad(subs(trajeqn1(1),q1,0))

traj2 = deg2rad(subs(trajeqn2(1),q2,0))

traj3 = deg2rad(subs(trajeqn3(1),q3,0))


function T = dhparam2matrix(theta, dist, radius, alpha)
    a = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), radius*cos(theta)];
    b = [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), radius*sin(theta)];
    c = [0, sin(alpha), cos(alpha), dist];
    d = [0, 0, 0, 1];
    T = [a;b;c;d];
end