function [xdot, u] = supply_model(x, u, nu_c, tau_ext, parameters)
% [xdot, u] = supply(x, u) returns the speed the time derivative xdot = A*x + B*tau
% of the state vector: x = [ x y psi u v r]'  for a supply vessel length L = 76 m.
%
% The model is only valid around zero speed (dynamic positioning) and 
% small speeds U = sqrt(u^2+v^2).
%
% u     = surge velocity                    (m/s)     
% v     = sway velocity                     (m/s)
% r     = yaw velocity                      (rad/s)
% x     = position in x-direction           (m)
% y     = position in y-direction           (m)
% psi   = yaw angle                         (rad)
%
% tau   = [X, Y, N]' control force/moment
%
% nu_c    = [ u_c v_c 0 ]'  where
%
% uc = current velocity in surge            (m/s)
% vc = current velocity in sway             (m/s)
%
% The external environmental force vector is denoted by tau_ext
%
% Reference : Fossen, T. I., S. I. Sagatun and A. J. Sorensen (1996)
%             Identification of Dynamically Positioned Ships
%             Journal of Control Engineering Practice CEP-4(3):369-376
%
% Author:     Thor I. Fossen
% Date:       12 July 2002
% Revisions:  24 February 2004 - Included missing mass scaling in the Bis transformation
%             12 October 2011 - Corrected T and Tinv, which were switched 
%             27 May 2019 - Added U as ouput
%             31 May 2019 - Included the rotation matrix in yaw

% Check of input and state dimensions
if (length(x)  ~= 9),error('x-vector must have dimension 9 !');end
if (length(u) ~=  2),error('u-vector must have dimension 2 !');end

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

% Rudder limitations
delta_max  = parameters.rudder.max_angle;    
Ddelta_max = parameters.rudder.max_angle_speed; 

% Propeller 
KT  = parameters.propeller.coefficients.KT;            
KQ  = parameters.propeller.coefficients.KQ;            
Dia = parameters.propeller.diameter;           
rho = parameters.propeller.density_of_water;           

% Input matrix
X_rudder = parameters.rudder.coefficients.X;
Y_rudder = parameters.rudder.coefficients.Y;
N_rudder = parameters.rudder.coefficients.N;
w        = parameters.propeller.wake_fraction_number; 


%% Ship model

M = parameters.ship.M;
Minv = parameters.ship.Minv;
D = parameters.ship.D;

r = x(3);

C = [0                               0                     -M(2, 2) * nu_r(2) - M(2, 3) * r
     0                               0                      M(1, 1) * nu_r(1)
     M(2, 2) * nu_r(2) - M(2, 3) * r -M(1, 1) * nu_r(1)     0];

%% Ship dynamics

% Propeller thrust and torque
thrust  = rho * Dia^4 * KT * abs(n) * n;
Q       = rho * Dia^5 * KQ * abs(n) * n; 

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
tau = [(1 - w)  -nu_r(1)^2 * X_rudder * delta
        0       -nu_r(1)^2 * Y_rudder
        0       -nu_r(1)^2 * N_rudder] * [thrust; delta];

eta_dot = Rzyx(0, 0, eta(3)) * nu;   
nu_dot = [nu(3) * vc -nu(3) * uc 0]' + M \ (tau + tau_ext - (D + C) * nu_r); 

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
 
