clc;
clear;

%% Parameters and data
load('supply.mat');
load('supplyABC.mat');
load('thrusters_sup.mat');
load('wind_coeff3.mat');
load('net_modeling.mat');

simulation_id = "100";

parameters                              = construct_parameters();

[waypoints_x, waypoints_y, waypoints_K] = generate_waypoints();
waypoints                               = [waypoints_x; waypoints_y];


%% Simulation
fprintf("Speed %d, span: %d\r\n", parameters.nominal.speed, parameters.nominal.span);
fprintf("Time to simulate: %f hours\r\n", parameters.end_time / 3600);
output = sim('model', parameters.end_time);

timespan = output.vessel1_data.Time;

data_vessel_1 = output.vessel1_data.Data;
data_vessel_2 = output.vessel2_data.Data;

%% Plot kinematic and actuator data
x1       = data_vessel_1(:, 1);
y1       = data_vessel_1(:, 2);
x2       = data_vessel_2(:, 1);
y2       = data_vessel_2(:, 2);
x_offset = data_vessel_2(:, 7);
y_offset = data_vessel_2(:, 8);

speed_vessel_1 = sqrt(data_vessel_1(:, 4).^2 + data_vessel_1(:, 5).^2);
speed_vessel_2 = sqrt(data_vessel_2(:, 4).^2 + data_vessel_2(:, 5).^2);

rudder_angle_vessel_1               = data_vessel_1(:, 10);
propeller_speed_vessel_1            = data_vessel_1(:, 11) * 60;
rudder_angle_commanded_vessel_1     = data_vessel_1(:, 12);
propeller_speed_commanded_vessel_1  = data_vessel_1(:, 13) * 60;

rudder_angle_vessel_2               = data_vessel_2(:, 12);
propeller_speed_vessel_2            = data_vessel_2(:, 13) * 60;
rudder_angle_commanded_vessel_2     = data_vessel_2(:, 14);
propeller_speed_commanded_vessel_2  = data_vessel_2(:, 15) * 60;

figure();
pathplotter(x1, y1, x2, y2, x_offset, y_offset, waypoints);
title("Position of Vessels");
legend(["Target path for Vessel 1", "Vessel 1", "Target path for Vessel 2", "Vessel 2"]);

saveas(gcf, sprintf("data/output/%s_position", simulation_id), "epsc")

figure();
subplot(3, 1, 1);
plot(timespan, speed_vessel_1, timespan, speed_vessel_2);
title("Speed");
xlabel("Time [s]");
ylabel("Speed [m/s]")
legend(["Vessel 1", "Vessel 2"]);
grid on;

subplot(3, 1, 2);
plot(timespan, propeller_speed_vessel_1, ...
     timespan, propeller_speed_commanded_vessel_1, ...
     timespan, propeller_speed_vessel_2, ...
     timespan, propeller_speed_commanded_vessel_2);
title("Propeller speed");
legend(["Vessel 1", "Commanded (Vessel 1)", "Vessel 2", "Commanded (Vessel 2)"]);
xlabel("Time [s]");
ylabel("Propeller speed [RPM]")
grid on;

subplot(3, 1, 3);
plot(timespan, rudder_angle_vessel_1, ...
     timespan, rudder_angle_commanded_vessel_1, ...
     timespan, rudder_angle_vessel_2, ...
     timespan, rudder_angle_commanded_vessel_2);
title("Rudder angle");
legend(["Vessel 1", "Commanded (Vessel 1)", "Vessel 2", "Commanded (Vessel 2)"]);
xlabel("Time [s]");
ylabel("Angle [rad]")
grid on;

saveas(gcf, sprintf("data/output/%s_data", simulation_id), "epsc")

%% Save net data

span              = sqrt((x1 - x2).^2 + (y1 - y2).^2);
netforce_vessel_1 = sqrt(data_vessel_1(:, 7).^2 + data_vessel_1(:, 8).^2);
netforce_vessel_2 = sqrt(data_vessel_2(:, 9).^2 + data_vessel_2(:, 10).^2);

net_data = zeros(3, length(timespan));
net_data(1, :) = timespan;
net_data(2, :) = span;
net_data(3, :) = netforce_vessel_1;

save(sprintf("data/output/%s.mat", simulation_id), "net_data");

%% Plot net data

figure();
subplot(2, 1, 1);
plot(timespan, span);
title("Net span");
xlabel("Time [s]");
ylabel("Span [m]");
grid on;

subplot(2, 1, 2);
plot(timespan, netforce_vessel_1, timespan, netforce_vessel_2);
title("Force from net on vessels");
legend(["Vessel 1", "Vessel 2"]);
xlabel("Time [s]");
ylabel("Force [N]")
grid on;

saveas(gcf, sprintf("data/output/%s_net", simulation_id), "epsc")