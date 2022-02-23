function [xdot, u] = ship_model(x, u, nu_c, tau_ext, parameters)
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
m  = parameters.ship.mass;               
Iz = parameters.ship.Iz;                % yaw moment of inertia (kg m^3)
xg = parameters.ship.xg;                % CG x-ccordinate (m)
L  = parameters.ship.length;             
B  = parameters.ship.beam;               
T  = parameters.ship.draft;              

% Added mass matrix
Xudot = parameters.added_mass.Xudot; 
Yvdot = parameters.added_mass.Yvdot;
Yrdot = parameters.added_mass.Yrdot; 
Nvdot = parameters.added_mass.Nvdot;
Nrdot = parameters.added_mass.Nrdot;

% Rudder limitations
delta_max  = parameters.rudder.max_angle;    
Ddelta_max = parameters.rudder.max_angle_speed; 

% Propeller 
KT  = parameters.propeller.coefficients.KT;            
KQ  = parameters.propeller.coefficients.KQ;            
Dia = parameters.propeller.diameter;           
rho = parameters.propeller.density_of_water;           

%% Mass matrix
Minv = inv(parameters.ship.M);

%% Input matrix
X_rudder = parameters.rudder.coefficients.X;
Y_rudder = parameters.rudder.coefficients.Y;
N_rudder = parameters.rudder.coefficients.N;

w        = parameters.propeller.wake_fraction_number; 

% This is the input matrix given by both the rudder actuator
% and the propeller
%
% For the rudder:
%
% Following Fossen (2021) p. 239 we have that the force and moment vector in
% surge, sway and yaw (X, Y, N) is given by:
%
% [-X_rudder * delta^2, -Y_rudder * delta -N_rudder * delta]
%
% For the propeller (p. 219 in Fossen 2021):
%
% [(1 - w) * u, 0, 0]
%
% Where we model the force in surge as the forward speed multiplied with
% a factor decided by the wake fraction number. This results in the speed 
% of the water going into the propeller. The force is then:
% thrust * (1 - w) * u

Bi = @(u_r, delta) [ (1 - w)  -u_r^2 * X_rudder * delta
                         0    -u_r^2 * Y_rudder
                         0    -u_r^2 * N_rudder];
             
%% Restoring forces (state dependent)

% Restoring force for rigid-body
CRB = m * nu_r(3) * [0 -1 -xg 
                     1  0  0 
                     xg 0  0];         

% Restoring force for added mass 
CA = [0                                  0    Yvdot * nu_r(2) + Yrdot * nu_r(3); 
      0                                  0    -Xudot * nu_r(1); 
      -Yvdot * nu_r(2) - Yrdot * nu_r(3) Xudot * nu_r(1) 0];    

C = CRB + CA;

%% Linear damping

% Natural Period for Surge, Sway and Yaw (s)
T1 = parameters.natural_periods.surge; 
T2 = parameters.natural_periods.sway; 
T6 = parameters.natural_periods.yaw;     

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
xLs = -L/2:dx:L/2;

for i = 1:length(xLs)
    xL = xLs(i);
    Ucf = abs(nu_r(2) + xL * nu_r(3)) * (nu_r(2) + xL * nu_r(3));

    Ycf = Ycf - 0.5 * rho * T * Cd_2D * Ucf * dx;           % Sway force
    Ncf = Ncf - 0.5 * rho * T * Cd_2D * xL * Ucf * dx;      % Yaw moment
end

d = -[Xns Ycf Ncf]';

%% Ship dynamics

% Propeller thrust and torque
thrust  = rho * Dia^4 * KT * abs(n) * n;
Q       = rho * Dia^5 * KQ * abs(n) * n; 

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
Im = parameters.propeller.propulsion.Im;
Tm = parameters.propeller.propulsion.Tm;
Km = parameters.propeller.propulsion.Km;                          

Qd = rho * Dia^5 * KQ * abs(n_c) * n_c;                 % Desired torque

Y = Qd / Km;                                            % Feedforward 
                                                        % moment controller
Qm_dot = 1/Tm * (-Qm + Km * Y);
n_dot =  1/Im * (Qm - Q);

%% Output
xdot = [nu_dot' eta_dot' delta_dot n_dot Qm_dot]';
u = [delta_c; n_c];
end
