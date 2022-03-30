function parameters = construct_parameters()

parameters.simulation.step_size = 0.05;

%% Ship 

m                       = 6000e3;     % mass (kg)
L = 76.2;

parameters.ship.mass    = m;                       
parameters.ship.length  = L;          % length (m)
parameters.ship.beam    = 21.8;          % beam (m)
parameters.ship.draft   = 8.9;           % draft (m)         


% Natural periods
parameters.natural_periods.surge    = 87.8131;

% Normalization variables
g    =  9.81;                           % acceleration of gravity (m/s^2)

T    = diag([1 1 1/L]);
Tinv = diag([1 1 L]);

% Model matricses
Mbis = [1.1274         0          0
             0    1.8902    -0.0744
             0   -0.0744     0.1278];

Dbis = [0.0358        0        0
             0        0.1183  -0.0124
             0       -0.0041   0.0308];
 

parameters.ship.M = (m*Tinv^2)*(T*Mbis*Tinv);
parameters.ship.Minv = inv(parameters.ship.M);
parameters.ship.D = (m*Tinv^2)*(sqrt(g/L)*T*Dbis*Tinv);


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

parameters.nomoto.Tn = 11.4117; 
parameters.nomoto.Kn = 0.0075;
parameters.nomoto.wb = 0.06;

%% Guidance

parameters.guidance.look_ahead = parameters.ship.length * 4;
parameters.guidance.kappa      = 0.01;


%% Current parameters
parameters.envloads.Current_dir     = -90;
parameters.envloads.Current_vel     = 0.2;

%% Wind parameters
parameters.envloads.Vw = 1;
parameters.envloads.betaVw = deg2rad(135);

%% Constant bearing

parameters.CB.Delta = 100;
parameters.CB.U_max = 0.5;

end

