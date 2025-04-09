syms q1 q2 q3 dq1 dq2 dq3

% function[A, B, C, D] = ss_model(q1, q2, q3, dq1, dq2, dq3)

l2 = 0.34;
% l3 = 0.37;

I1 = 0.2473;
I2 = 0.1246;
I3 = 0.0402;

m2 = 1.84;
m3 = 3.46;

r2 = 0.097;
r3 = 0.150;

% g0 = 9.8;       

a1 = m2*r2*r2 + m3*l2*l2;
a2 = m3*r3*r3;
a3 = m3*r3*l2;
% b1 = (m2*r2 + m3*l2) * g0;
% b2 = m3*r3*g0;

M = [I1 + a1*cos(q2)*cos(q2) + a2*cos(q2+q3)*cos(q2+q3) + 2*a3*cos(q2)*cos(q2+q3), 0,                           0;
     0,                                                                            I2 + a1 + a2 + 2*a3*cos(q3), a2 + a3*cos(q3);
     0,                                                                            a2 + a3*cos(q3),             I3 + a2];                           

M = vpa(M, 4)

N = [-0.5*a1*dq2*sin(2*q2) - 0.5*a2*(dq2+dq3)*sin(2*q2+2*q3) - a3*dq2*sin(2*q2+q3) - a3*dq3*cos(q2)*sin(q2+q3), -0.5*a1*dq1*sin(2*q2) - 0.5*a2*dq1*sin(2*q2 + 2*q3) - a3*dq1*sin(2*q2+q3), -0.5*a1*dq1*sin(2*q2+2*q3) - a3*dq1*cos(q2)*sin(q2+q3);
     0.5*a1*dq1*sin(2*q2) + 0.5*a2*dq1*sin(2*q2 + 2*q3) + a3*dq1*sin(2*q2+q3),                                  -a3*dq3*sin(q3),                                                           -a3*(dq2+dq3)*sin(q3);
     0.5*a1*dq1*sin(2*q2+2*q3) + a3*dq1*cos(q2)*sin(q2+q3),                                                     -a3*dq2*sin(q3),                                                           0]; 

N = vpa(N, 4)

% g = [0;
%     b1*cos(q2) + b2*cos(q2+q3);
%     b2*cos(q2 + q3)];

A = [zeros(3), -eye(3)
    zeros(3), -simplify(inv(M)*N)];
B = [zeros(3); eye(3)];
C = eye(6);
D = zeros(3);

% end

% disp(A);
% disp(B);

% disp(-vpa(inv(M)*N, 2));