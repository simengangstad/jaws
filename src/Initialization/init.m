%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% init()                                                                  %
%                                                                         %                                                 %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;

load('supply.mat');
load('supplyABC.mat');
load('thrusters_sup.mat')
load('wind_coeff3.mat');
% Initial position x, y, z, phi, theta, psi
eta0 = [0,0,0,0,0,0]';
% Initial velocity u, v, w, p, q, r
nu0 = [0,0,0,0,0,0]';

%% 
massMatrix = inv(vesselABC.Minv);
massvector = [massMatrix(1,1),massMatrix(2,2),massMatrix(6,6)]; %the mass in surge, sway and yaw
% Tn = [10 30 20]*2.5; %ORIGINAL
% Tn = [100 100 100]*1e10;  working without errors
Tn = [0.1 0.1 10]*1e6;
Tp = [10 10 10];
omega_n = 2*pi./Tn;
omega_t = 2*pi./Tp;
Ref_freq = 0.05;

Kp = omega_n.^2 .* massvector*1e6; % PID tuning method is called "Ziegler Nichols method" 
Ki = [0 0 0];%Kp./Tn;
Kd = 0.7 * 2 .* omega_n .* massvector*1e-3; %0.7 * 2 .* omega_n .* massvector; %WILL PROBABLY NEED FOR CURRENTS. SET TO APPROX 1e3

%wind parameters
wind_dir_mean = 170;          
k = 0.003;
mu = 0.001;
U10 = 15;   %the NORSOK spectrum contains more energy at lower frequencies U10=10
z = 3;
h = 1;
w = 0.005;

z0 = 10*exp(-2/(5*sqrt(k)));
Umean = U10 * 5/2 * sqrt(k) * log(z/z0);

nfreq = 100;
L = 1800;
k = 0.0026;

for i = 1:nfreq
    f(i) = 0.01 + (i-1)/(nfreq-1) * 0.99;
    ftilde = L*f(i)/U10;
    S(i) = (4*k*L*U10) / ((2+ftilde^2)^(5/6));
end
    
phi = 2*pi * rand(nfreq,1);
    
for t = 0:4000
    Ug(t+1) = 0;
    for i = 1:nfreq
        Ug(t+1) = Ug(t+1) + sqrt(2*S(i)*(f(2)-f(1))) * cos(2*pi*f(i)*t + phi(i));
    end
end
Windgust_timeseries = timeseries(Ug);

%Current parameters
Current_dir     = -90;
Current_vel     = 0.2;