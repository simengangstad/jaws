clear;
parameters = construct_parameters();

M = parameters.ship.M;
D = parameters.ship.D;

%% Sway and yaw

Ms = M(2:3, 2:3);
Ds = D(2:3, 2:3);
C = [0, 1];

% System
A = -Ms \ Ds;
B = inv(Ms);

[b, a] = ss2tf(A, B, C, [0, 0], 2);

K = b(3);
T_3 = b(2) / b(3);

r = roots(a);
T_1 = -1/r(1);
T_2 = -1/r(2);

K = K / (-r(1) * -r(2));

% Build first order nomoto model

disp("First order nomoto:")
disp("T: ");
disp(T_1 + T_2 - T_3);

disp("K: ");
disp(K);
 

%% Surge
Ms = M(1, 1);
Ds = D(1, 1);

% System
A = -Ms \ Ds;
B = inv(Ms);

[b, a] = ss2tf(A, B, 1, 0);

K = b(2)/a(2);
T = 1/a(2);

disp("Surge")

disp("T: ");
disp(T);

disp("K: ");
disp(K);