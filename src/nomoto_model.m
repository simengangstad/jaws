L    =  76.2;           % length of ship (m)
g    =  9.8;            % acceleration of gravity (m/s^2)
mass = 6000e3;          % mass (kg)

T    = diag([1 1 1/L]);
Tinv = diag([1 1 L]);

Mbis = [1.1274         0          0
             0    1.8902    -0.0744
             0   -0.0744     0.1278];

Dbis = [0.0358        0        0
             0        0.1183  -0.0124
             0       -0.0041   0.0308];

  
M = (mass*Tinv^2)*(T*Mbis*Tinv);
D = (mass*Tinv^2)*(sqrt(g/L)*T*Dbis*Tinv);

% Only look at sway and yaw

M = M(2:3, 2:3);
D = D(2:3, 2:3);
C = [0, 1];

% System
A = -M \ D;
B = inv(M);

[b, a] = ss2tf(A, B, C, [0, 0], 2);

K = b(3);
T_3 = b(2) / b(3);

r = roots(a);
T_1 = -1/r(1);
T_2 = -1/r(2);

T_1
T_2
T_3

% Build first order nomoto model

T = T_1 + T_2 - T_3
 