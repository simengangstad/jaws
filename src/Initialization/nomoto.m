%% C_RB & C_A
syms u_d
syms u v r m xg real
nu = [u; v; r];

C_RB = [0               -m * r      -xg * m * r;
        m * r            0          0;
        xg * m * r       0          0];

v = 0;
r = 0;
u = u_d;

C_RB_star = subs(jacobian((C_RB*nu)', nu'));
    
syms u v r Xudot Yvdot Yrdot real
nu = [u; v; r];

a_1 = Xudot * u;
a_2 = Yvdot * v + Yrdot * r;


C_A = [0    0    a_2;
       0    0   -a_1;
       -a_2 a_1  0];

v = 0;
r = 0;
u = u_d;

C_A_star = subs(jacobian((C_A*nu)', nu'));

%% D
syms Xu Yv Yr Nv Nr

D = - [Xu 0 0;
       0 Yv Yr;
       0 Nv Nr];

%% M_RB & M_A

syms m Iz
M_RB = [m 0 0;
        0 m m * xg;
        0 m * xg Iz];
    
syms Xudot Yvdot Yrdot Nvdot Nrdot
M_A = - [Xudot 0     0;
         0     Yvdot Yrdot;
         0     Nvdot Nrdot];
     
%% M & N
M = M_RB + M_A;
M = M(2:3, 2:3);

N = C_RB_star + C_A_star + D;
N = N(2:3, 2:3);


%% Substitude all variables

m = 17.0677e6;          % mass (kg)
L = 161;                % length (m)
rho = 1025;             % density of water (m/s^3)
Iz = 2.1732e10;         % yaw moment of inertia (kg m^3)
xg = -3.7;              % CG x-ccordinate (m)

Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;

T1 = 20; T2 = 20; T6 = 10;
Xu = -(m-Xudot)/T1;
Yv = -(m-Yvdot)/T2;
Nr = -(Iz-Nrdot)/T6;
Yr = 0;
Nv = 0;

u_d = 7;

M = double(subs(M));
N = double(subs(N));

M = [5.3122e6 0 0; 0 8.2831e6 0; 0 0 3.7454e9];
N = [5.0242e4 0 0; 0 2.7229e5 -4.3933e6; 0 -4.3933e6 4.1894e8];


%% Find input coeffiecients

% rudder coefficients
b = 2;
AR = 8;
CB = 0.8;

lambda = b^2 / AR;
tR = 0.45 - 0.28*CB;
CN = 6.13*lambda / (lambda + 2.25);
aH = 0.75;
xH = -0.4 * L;
xR = -0.5 * L;

t_thr = 0.05;                                        % thrust deduction number
X_delta2 = 0.5 * (1 - tR) * rho * AR * CN;           % rudder coefficients (Section 9.5)
Y_delta = 0.25 * (1 + aH) * rho * AR * CN; 
N_delta = 0.25 * (xR + aH*xH) * rho * AR * CN;   

b_input = 2 * u_d * [-Y_delta -N_delta]';

%% State space

A = -M \ N;
B = M \ b_input;
C = [0 1];
D = 0;

[num, den] = ss2tf(A, B, C, D);

poles = roots(den);

T_1 = -1/poles(1);
T_2 = -1/poles(2);
T_3 = num(2)/num(3);
K = num(3)/(poles(1)*poles(2))
T = T_1 + T_2 - T_3