function parameters = construct_parameters()

parameters.simulation.step_size = 0.05;

%% Ship 

m                       = 17.0677e6;     % mass (kg)
Iz                      = 2.1732e10;     % yaw moment of inertia (kg m^3)
xg                      = -3.7;          % CG x-ccordinate (m)

parameters.ship.mass    = m;             
parameters.ship.Iz      = Iz;            
parameters.ship.xg      = xg;            
parameters.ship.length  = 161;           % length (m)
parameters.ship.beam    = 21.8;          % beam (m)
parameters.ship.draft   = 8.9;           % draft (m)         

% Added mass matrix
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;

parameters.added_mass.Xudot = Xudot;
parameters.added_mass.Yvdot = Yvdot;
parameters.added_mass.Yrdot = Yrdot;
parameters.added_mass.Nvdot = Nvdot;
parameters.added_mass.Nrdot = Nrdot;

% Natural periods
parameters.natural_periods.surge    = 20;
parameters.natural_periods.sway     = 20;
parameters.natural_periods.yaw      = 10;


% Rigid-body mass matrix
MRB = [parameters.ship.mass   0       0 
       0   m       m * xg
       0   m * xg  Iz];

% Added mass matrix
MA = -[Xudot    0       0; 
       0        Yvdot   Yrdot; 
       0        Nvdot   Nrdot];

parameters.ship.M = MRB + MA;
parameters.ship.Minv = inv(parameters.ship.M);

% Hoerner for damping

parameters.ship.Cd_2D = Hoerner(parameters.ship.beam, parameters.ship.draft);

%% Rudder

% Limitations
parameters.rudder.max_angle       = 70 * pi/180;   
parameters.rudder.max_angle_speed = 5  * pi/180;   

% Coefficients

% The following is outlined in Fossen (2021) p. 237-240
% 
% We want to have an estimate for the surge and sway forces as well as the 
% yaw moment caued by the rudder. We have the following parameters
%
% rho       - Density of SEA water (so not 1000 kg/m^3, but ~1025 kg/m^3)
% UR        - Rudder inflow speed
% AR        - Area of rudder 
% alpha_R   - Effective rudder angle
% CB        - Block coefficient, given by nabla / (L * B * T)
% tR        - Additional drag
% CN        - Drag/force coefficient
% aH        - Rudder force increase factor 
% xH        - Longitudinal coordinate of additional lateral force

rho                 = 1025;
rudder_height       = 2;
AR                  = 8;
rudder_aspect_ratio = rudder_height^2 / AR;
CB                  = 0.8; 
tR                  = 0.45 - 0.28 * CB;
CN                  = 6.13 * rudder_aspect_ratio / (rudder_aspect_ratio + 2.25);
aH                  = 0.75;
xH                  = -0.4 * parameters.ship.length;
xR                  = -0.5 * parameters.ship.length;

% To get an estimate of the surge and sway forces as well as yaw moment caused
% by the rudder we assume delta being small such that sin(delta) is 
% approximately delta
%
% X_rudder = -0.5 * (1 - tR) * rho * UR^2 * AR * CN * sin^2(delta) 
%          ~ -0.5 * (1 - tR) * rho * UR^2 * AR * CN * delta^2
%
% Y_rudder = -0.25 * (1 + aH) * rho * UR^2 * AR * CN * sin(2 * delta)
%          ~ -0.5  * (1 + aH) * rho * UR^2 * AR * CN * delta
%
% N_rudder = -0.25 * (xR + aH * xH) * rho * UR^2 * AR * CN * sin(2*delta)
%          ~ -0.5  * (xR + aH * xH) * rho * UR^2 * AR * CN * delta
%
% This results in the following coefficients:

% Note that UR^2 is omitted here as it is calculated in simulation as it
% is the speed and we don't have access to it here
parameters.rudder.coefficients.X = 0.5 * (1 - tR) * rho * AR * CN;
parameters.rudder.coefficients.Y = 0.5  * (1 + aH) * rho * AR * CN;
parameters.rudder.coefficients.N = 0.5  * (xR + aH * xH) * rho * AR * CN;

%% Propeller
PD      = 1.5;
AEAO    = 0.65;
nBlades = 4;
[KT,KQ] = wageningen(0, PD, AEAO, nBlades);       % Propeller coefficients

parameters.propeller.wake_fraction_number = 0.05; 
parameters.propeller.coefficients.KT      = KT;
parameters.propeller.coefficients.KQ      = KQ;
parameters.propeller.diameter             = 3.3;     
parameters.propeller.density_of_water     = rho; % Note: sea water, so not 
                                                 % 1000 kg/m^3 but 1025 kg/m^3

parameters.propeller.propulsion.Im = 1e5;
parameters.propeller.propulsion.Tm = 10;
parameters.propeller.propulsion.Km = 0.6;

%% Nomoto model

parameters.nomoto.Tn = 169.5493; 
parameters.nomoto.Kn = 0.0075;
parameters.nomoto.wb = 0.06;

%% Guidance

parameters.guidance.look_ahead = 1800;
parameters.guidance.kappa      = 1.5;

%% Controllers

% Surge controller
% surge_lambda = 1;
% 
% % Surge reference model
% surge_omega_n = 0.01;
% surge_zeta = 1; 
%    
% % Course controller
% yaw_omega_n = 0.06;
% yaw_T = 169;



end

