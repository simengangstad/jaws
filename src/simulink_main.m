clear;
load('supply.mat');
load('supplyABC.mat');
load('thrusters_sup.mat');
load('wind_coeff3.mat');

parameters = construct_parameters();
[waypoints_x, waypoints_y, waypoints_K] = generate_waypoints();
waypoints = [waypoints_x; waypoints_y];

output = sim('ship', 6 * 3600);

x1 = output.simulation_data1.Data(:, 1);
y1 = output.simulation_data1.Data(:, 2);

data2 = reshape(output.simulation_data2.Data, 8, [])';

x2 = data2(:, 1)';
y2 = data2(:, 2)';

x_offset = data2(:, 7)';
y_offset = data2(:, 8)';

pathplotter(x1, y1, x2, y2, x_offset, y_offset, waypoints, [])
