%% Ship parameters
PD = 1.5;
AEAO = 0.65;
nBlades = 4;
[KT,KQ] = wageningen(0,PD,AEAO,nBlades); 

%% Controllers

% Surge controller
surge_lambda = 1;

% Surge reference model
surge_omega_n = 0.01;
surge_zeta = 1; 
   
% Course controller
yaw_omega_n = 0.06;
yaw_T = 169;




