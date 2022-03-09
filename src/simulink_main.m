clear;

parameters = construct_parameters();
load('supply.mat');
load('supplyABC.mat');
load('thrusters_sup.mat');
load('wind_coeff3.mat');
output = sim('ship', 6000);

x = output.simulation_data.Data(:, 1);
y = output.simulation_data.Data(:, 2);

pathplotter(x, y)