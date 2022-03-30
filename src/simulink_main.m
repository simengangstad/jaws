clear;
load('supply.mat');
load('supplyABC.mat');
load('thrusters_sup.mat');
load('wind_coeff3.mat');
load('net_modeling.mat');

parameters = construct_parameters();
[waypoints_x, waypoints_y, waypoints_K] = generate_waypoints();
waypoints = [waypoints_x; waypoints_y];

output = sim('ship', 1 * 3600);

%% Plot path

timespan = output.simulation_data1.Time;

data_vessel_1 = output.simulation_data1.Data;
x1 = data_vessel_1(:, 1);
y1 = data_vessel_1(:, 2);

% Need to reshape for some reason for data 2, might be because we have that
% mux at the end
data_vessel_2 = output.simulation_data2.Data;

x2       = data_vessel_2(:, 1);
y2       = data_vessel_2(:, 2);
x_offset = data_vessel_2(:, 7);
y_offset = data_vessel_2(:, 8);

pathplotter(x1, y1, x2, y2, x_offset, y_offset, waypoints, [])

%% Plot statistics
figure()

span = sqrt((x1 - x2).^2 + (y1 - y2).^2);
plot(timespan, span);
title("Span");
xlabel("Time [s]");
ylabel("Span [m]");
grid on;