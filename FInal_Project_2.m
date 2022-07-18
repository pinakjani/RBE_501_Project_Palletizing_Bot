DATA_SIZE = 106
syms m1 m2 m3 real
syms q1 q2 q3 real
syms g real
syms q_dot_1 q_dot_2 q_dot_3 q_ddot_1 q_ddot_2 q_ddot_3 real
% theta, ai_1, di, alpha
joint_2_offset = (pi/2) - 0.49;
joint_3_offset = (pi/2) + 0.57;
DH = [q1,0,25.,0; % Joint1 
      q2+joint_2_offset,14.07,-31.85,pi/2;% Joint 2
      q3+joint_3_offset,200,0,pi;% Joint 3
      0,200,0,0;% Joint EE
      0,12.5,0,0;%Base -> Link1 CoM
      0,110.86,-9.5,-pi/2;%Joint 1 -> Link2 CoM
      0,14.99,122.2,0];%Joint2 -> Link3 CoM

I1_value = [419415.58, 336026.6,-223672.96;
            336026.60,865880.11,-111006.92;
            -223672.96,-111006.92,898860.72];

I2_value = [2349278.50,930057.87,-38028.71;
            930057.87,436281.52,-88515.72;
            -38028.71,-88515.72,2776226.57];

I3_value = [9665842.37,-1532433.01,185580.64;
            -1532433.01,1118334.23,-1228704.69;
            185580.64,-1228704.69,10298383.57];

for c = 1:7
    Transforms(:,:,c) = ModifyedDHparam2matrix(DH(c,1),DH(c,2),DH(c,3),DH(c,4));
end
% Forward Kinamatics test
T0EE = Transforms(:,:,1)*Transforms(:,:,2)*Transforms(:,:,3)*Transforms(:,:,4);
% double(subs(T0EE,[q1,q2,q3],[0,pi/2,pi/2]))


mass1_T = Transforms(:,:,1)*Transforms(:,:,5);
mass1_pos = mass1_T(1:3,4);
Jv1 = [diff(mass1_pos, q1) [0;0;0] [0;0;0]];
Jw1 = [[0;0;1] [0;0;0] [0;0;0]];
R1 = mass1_T(1:3, 1:3);

mass2_T = Transforms(:,:,1)*Transforms(:,:,2)*Transforms(:,:,6);
mass2_pos = mass2_T(1:3,4);
Jv2 = [diff(mass2_pos, q1) diff(mass2_pos, q2) [0;0;0]];
Jw2 = [[0;0;1] [0;-1;0] [0;0;0]];
R2 = mass2_T(1:3, 1:3);

mass3_T = Transforms(:,:,1)*Transforms(:,:,2)*Transforms(:,:,3)*Transforms(:,:,7);
mass3_pos = mass3_T(1:3,4);
Jv3 = [diff(mass3_pos, q1) diff(mass3_pos, q2) diff(mass3_pos, q3)];
Jw3 = [[0;0;1] [0;-1;0] [0;-1;0]];
R3 = mass3_T(1:3, 1:3);

Kw1 = Jw1.' * R1*I1_value*R1.'*Jw1;
Kw2 = Jw2.' * R2*I2_value*R2.'*Jw2;
Kw3 = Jw3.' * R3*I3_value*R3.'*Jw3;

K1 = m1*(Jv1.')*Jv1 + Kw1;
K2 = m2*(Jv2.')*Jv2 + Kw2;
K3 = m3*(Jv3.')*Jv3 + Kw3;

% D 3x3 matrix
D = simplify(K1 + K2 + K3);

K = simplify((1/2)*[q_dot_1 q_dot_2 q_dot_3]*D*[q_dot_1; q_dot_2; q_dot_3]);
a = [1 2 3];
thetas = [q1, q2, q3];
theta_dots = [q_dot_1, q_dot_2, q_dot_3];
corilois_ijk = q1.*ones([3,3,3]);
for i = a
    for j = a
        for k = a
            qi = thetas(i);
            qj = thetas(j);
            qk = thetas(k);
            corilois_ijk(i,j,k) = (1/2)*(diff(D(k,j), qi) +...
                diff(D(k,i), qj) - diff(D(i,j), qk));
        end
    end
end
C = q1*ones([3,3]);
for k = a
    for j = a
        sum = 0;
        for i = a
            qdi = theta_dots(i);
            sum = sum + corilois_ijk(i,j,k)*qdi;
        end
        C(k,j) = sum;
    end
end
P1 = m1*g*mass1_pos(3);
P2 = m2*g*mass2_pos(3);
P3 = m3*g*mass3_pos(3);
P = P1+P2+P3;
g1 = diff(P, q1);
g2 = diff(P, q2);
g3 = diff(P, q3);
G = [g1; g2; g3];
Lagrange_Tau = simplify(expand(D*[q_ddot_1;q_ddot_2;q_ddot_3] +...
    C*[q_dot_1; q_dot_2; q_dot_3] + G))
for i = 1:DATA_SIZE
    % Only have to run this part to get new values
    
    tau_subs = simplify(subs(Lagrange_Tau,[g,m1,m2,m3,q1,q2,q3,q_dot_1,q_dot_2,q_dot_3,q_ddot_1,q_ddot_2,q_ddot_3],[9.8,125.62,79.99,429.4837,z1(i),z2(i),z3(i),z4(i),z5(i),z6(i),0,0,0]));
    result = double(tau_subs) * 0.0000098;
    % this is in Newton Meters
    Motor1_T(i) = result(1)/100;
    Motor2_T(i) = result(2)/100;
    Motor3_T(i) = result(3)/100;
    i
    % end of loop part
end
subplot(2,3,1)
plot(1:DATA_SIZE,z1(1:DATA_SIZE),1:DATA_SIZE,z2(1:DATA_SIZE),1:DATA_SIZE,z3(1:DATA_SIZE))
legend("Motor 1 Position","Motor 2 Position","Motor 3 Position")
title("Motor Positions")
subplot(2,3,2)
plot(1:DATA_SIZE,torq1(1:DATA_SIZE),1:DATA_SIZE,Motor1_T(1:DATA_SIZE))
legend("Torque 1 SIM","Torque 1 EQU")
title("Motor 1 Torque Comparison")
subplot(2,3,4)
plot(1:DATA_SIZE,torq2(1:DATA_SIZE),1:DATA_SIZE,Motor2_T(1:DATA_SIZE))
legend("Torque 2 SIM","Torque 2 EQU")
title("Motor 2 Torque Comparison")
subplot(2,3,5)
plot(1:DATA_SIZE,torq3(1:DATA_SIZE),1:DATA_SIZE,Motor3_T(1:DATA_SIZE))
legend("Torque 3 SIM","Torque 3 EQU")
title("Motor 3 Torque Comparison")
subplot(2,3,6)
(1:DATA_SIZE,z4(1:DATA_SIZE),1:DATA_SIZE,z5(1:DATA_SIZE),1:DATA_SIZE,z6(1:DATA_SIZE))
legend("Motor 1 Velocity","Motor 2 Velocity","Motor 3 Velocity")
title("Motor Velocitys")

function T = ModifyedDHparam2matrix(theta, ai_1, di, alpha)
    a = [cos(theta), -sin(theta), 0, ai_1];
    b = [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -di*sin(alpha)];
    c = [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), di*cos(alpha)];
    d = [0, 0, 0, 1];
    T = [a;b;c;d];
end
