%% Current Direction
Current_d = 270; % direction(degree)
Current_V = 0.2; % velocity

%% Define the setpoints
Setpoint = [0,0,0];
 
% Tunning parameters
%% Reference model parameters
zeta = 1;
w_n = [0.065,0.065,0.08];

%% Controller
Kp = [1e+6,1.6e+6,2.5e+08];
Kd = [1e+7,1.2e+7,3e+9];
Ki = [1.2e+04,3e+04,1.2e+05];

% Observer Data
%% Low frequency control plant
% Vessel model
M=[7.0101e6 0 0;
   0 7.8784e6 -2.5955e6;
   0 -2.5955e6 3.57e9];
D=[2.6486e5 0 0;
   0 8.8164e5 0;
   0 0 3.3774e8];
% Bias model
T_b=diag([1000,1000,1000]);
E_b=diag([1,1,1]);

%% Wave frequency control plant
Omega=diag([2*pi/9,2*pi/9,2*pi/9]);
Lambda=diag([0.07,0.07,0.07]);
K_omega=diag([0.5,0.5,0.01]);

% State space form matrices of WF motions
A_omega=[zeros(3,3) eye(3,3);
                      -Omega^2 -2*Lambda*K_omega];
E_omega=[zeros(3,3);
                      K_omega];
C_omega=[zeros(3,3) eye(3,3)];

%% State space form matrics of control plant model
B=[zeros(12,3);
    inv(M)];
E=[E_omega zeros(6,3);
        zeros(3,6);
        zeros(3,3) E_b;
        zeros(3,6)];
H=[C_omega eye(3,3) zeros(3,6)];

% Covariance Matrices
%% Initialization of Kalman Filter
x_bar0=zeros(15,1);
P_bar0=diag([0,0,0,10,10,1,0,0,0,1e10,1e10,1e12,0,0,0]);

%% Process noise Q (15*15)
% Standard deviation of disturbance w
w=[1; 1; 0.1;1e3; 1e3; 1e4];
Q=diag((E*w)*(E*w)');

%% Measurement noise R (3*3)
R=diag([1,1,0.01]);

%%  Capability plot
% direction_environment = pi+90/180*pi;
% Current_d = direction_environment / pi * 180; % direction(degree)

%% Nonlinear observer
omega_oi=2*pi/9;
omega_ci=1.1;
zeta_i=0.1;
zeta_ni=1;

k_1i=-2*omega_ci*(zeta_ni-zeta_i)/omega_oi;
k_2i=2*omega_oi*(zeta_ni-zeta_i);
k_3i=omega_ci;

K1=[k_1i*eye(3,3);
    k_2i*eye(3,3)];
K2=k_3i*eye(3,3);
K3=diag([0.01 0.01 0.001]);
K4=10*K3;


