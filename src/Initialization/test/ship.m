function [xdot, u] = ship(x, u, nu_c, tau_ext)
% xdot = ship(x, u) returns the time derivative of the state vector: 
% x = [ u v r x y psi delta n Qm]' for a ship with L = 161 m where:
%
% u     = surge velocity, must be positive  (m/s)    
% v     = sway velocity                     (m/s)
% r     = yaw velocity                      (rad/s)
% x     = position in x-direction           (m)
% y     = position in y-direction           (m)
% psi   = yaw angle                         (rad)
% delta = actual rudder angle               (rad)
% n     = actual shaft velocity             (rpm)
% Qm    = torque produced by the engine     (Nm)
%
% The input vector is :
%
% u       = [ delta_c  n_c ]'  where
%
% delta_c = commanded rudder angle          (rad)
% n_c     = commanded shaft velocity        (rpm)
% The current vector is : 
%
% nu_c    = [ u_c v_c 0 ]'  where
%
% uc = current velocity in surge            (m/s)
% vc = current velocity in sway             (m/s)
%
% The external environmental force vector is denoted by tau_ext

% Check of input and state dimensions
if (length(x)~= 9),error('x-vector must have dimension 8 !');end
if (length(u)~= 2),error('u-vector must have dimension 2 !');end

%% Dimensional states and input
delta_c = u(1); 
n_c     = u(2);

nu    = x(1:3);
eta   = x(4:6);
delta = x(7);
n     = x(8); 
Qm    = x(9);

nu_r = nu - nu_c;
uc = nu_c(1);
vc = nu_c(2);

%% Parameters

% Ship parameters 
m = 17.0677e6;          % mass (kg)
Iz = 2.1732e10;         % yaw moment of inertia (kg m^3)
xg = -3.7;              % CG x-ccordinate (m)
L = 161;                % length (m)
B = 21.8;               % beam (m)
T = 8.9;                % draft (m)
KT = 0.6367;            % propeller coefficients
KQ = 0.1390;            
Dia = 3.3;              % propeller diameter (m)
rho = 1025;             % density of water (m/s^3)

% Rudder limitations
delta_max  = 40 * pi/180;        % max rudder angle      (rad)
Ddelta_max = 5  * pi/180;        % max rudder derivative (rad/s)

% Added mass matrix
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;

% Rudder coefficients
b = 2;
AR = 8;
CB = 0.8;

lambda = b^2 / AR;
tR = 0.45 - 0.28*CB;
CN = 6.13*lambda / (lambda + 2.25);
aH = 0.75;
xH = -0.4 * L;
xR = -0.5 * L;

%% Mass matrix

% Rigid-body mass matrix
MRB = [ m 0    0 
        0 m    m*xg
        0 m*xg Iz ];

% Added mass matrix
MA = -[Xudot 0 0; 
       0 Yvdot Yrdot; 
       0 Nvdot Nrdot];

Minv = inv(MRB + MA);

%% Input matrix
t_thr = 0.05;                                        % Thrust deduction number
X_delta2 = 0.5 * (1 - tR) * rho * AR * CN;           % Rudder coefficients
Y_delta = 0.25 * (1 + aH) * rho * AR * CN; 
N_delta = 0.25 * (xR + aH*xH) * rho * AR * CN;   

Bi = @(u_r,delta) [ (1-t_thr)  -u_r^2 * X_delta2 * delta
                        0      -u_r^2 * Y_delta
                        0      -u_r^2 * N_delta            ];
             
%% Restoring forces (state dependent)

% Restoring force for rigid-body
CRB = m * nu_r(3) * [ 0 -1 -xg 
                    1  0  0 
                    xg 0  0  ];         

% Restoring force for added mass 
CA = [0 0 Yvdot * nu_r(2) + Yrdot * nu_r(3); 
      0 0 -Xudot * nu_r(1); 
      -Yvdot * nu_r(2) - Yrdot * nu_r(3) Xudot * nu_r(1) 0];    

C = CRB + CA;

%% Linear damping

% Natural Period for Surge, Sway and Yaw (s)
T1 = 20; 
T2 = 20; 
T6 = 10;     

Xu = -(m - Xudot) / T1;
Yv = -(m - Yvdot) / T2;
Nr = -(Iz - Nrdot) / T6;
D = diag([-Xu -Yv -Nr]); 

%% Nonlinear damping (state dependent)
eps = 0.001;                    % Add small number to ensure Cf is well
                                % defined at u = 0
CR = 0;
k = 0.1;
S = B*L + 2*T*(B+L);
v = 1e-6;
Rn = L / v * abs(nu_r(1));
Cf = 0.075 / (log10(Rn) - 2 + eps)^2 + CR;
Xns = -0.5 * rho * S * (1+k) * Cf * abs(nu_r(1)) * nu_r(1);


% Strip theory: cross-flow drag integrals
Ycf = 0; 
Ncf = 0;
dx = L/10;                                                  % 10 strips
Cd_2D = Hoerner(B, T);

for xL = -L/2:dx:L/2
    Ucf = abs(nu_r(2) + xL * nu_r(3)) * (nu_r(2) + xL * nu_r(3));

    Ycf = Ycf - 0.5 * rho * T * Cd_2D * Ucf * dx;           % Sway force
    Ncf = Ncf - 0.5 * rho * T * Cd_2D * xL * Ucf * dx;      % Yaw moment
end

d = -[Xns Ycf Ncf]';

%% Ship dynamics

% Propeller thrust and torque
thrust = rho * Dia^4 * KT * abs(n) * n;
Q = rho * Dia^5 * KQ * abs(n) * n; 

tau = Bi(nu_r(1), delta) * [thrust; delta];

nu_dot = [nu(3) * vc -nu(3) * uc 0]' + Minv * (tau_ext + tau - (C + D) * nu_r - d); 
eta_dot = Rzyx(0, 0, eta(3)) * nu;    

%% Rudder saturation and dynamics
if abs(delta_c) >= delta_max
    delta_c = sign(delta_c) * delta_max;
end

delta_dot = delta_c - delta;
if abs(delta_dot) >= Ddelta_max
    delta_dot = sign(delta_dot) * Ddelta_max;
end    

%% Propeller dynamics
Im = 1e5; Tm = 10; Km = 0.6;                            % Propulsion 
                                                        % parameters

Qd = rho * Dia^5 * KQ * abs(n_c) * n_c;                 % Desired torque

Y = Qd/Km;                                              % Feedforward 
                                                        % moment controller
Qm_dot = 1/Tm * (-Qm + Km * Y);
n_dot =  1/Im * (Qm - Q);

%% Output
xdot = [nu_dot' eta_dot' delta_dot n_dot Qm_dot]';
u = [delta_c; n_c];
end