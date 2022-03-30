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

timespan = output.vessel1_data.Time;

data_vessel_1 = output.vessel1_data.Data;
x1 = data_vessel_1(:, 1);
y1 = data_vessel_1(:, 2);

% Need to reshape for some reason for data 2, might be because we have that
% mux at the end
data_vessel_2 = output.vessel2_data.Data;

x2       = data_vessel_2(:, 1);
y2       = data_vessel_2(:, 2);
x_offset = data_vessel_2(:, 7);
y_offset = data_vessel_2(:, 8);

pathplotter(x1, y1, x2, y2, x_offset, y_offset, waypoints, [])

%% Plot statistics
figure()

speed_vessel_1 = sqrt(data_vessel_1(:, 4).^2 + data_vessel_1(:, 5).^2);
speed_vessel_2 = sqrt(data_vessel_2(:, 4).^2 + data_vessel_2(:, 5).^2);
span           = sqrt((x1 - x2).^2 + (y1 - y2).^2);
netforce_vessel_1 = sqrt(data_vessel_1(:, 7).^2 + data_vessel_1(:, 8).^2);
netforce_vessel_2 = sqrt(data_vessel_2(:, 9).^2 + data_vessel_2(:, 10).^2);

rudder_angle_vessel_1       = data_vessel_1(:, 10);
propeller_speed_vessel_1    = data_vessel_1(:, 11) * 60;
rudder_angle_vessel_2       = data_vessel_2(:, 12);
propeller_speed_vessel_2    = data_vessel_2(:, 13) * 60;


subplot(3, 2, 1);
plot(timespan, span);
title("Net span");
xlabel("Time [s]");
ylabel("Span [m]");
grid on;

subplot(3, 2, 2);
plot(timespan, netforce_vessel_1, timespan, netforce_vessel_2);
title("Force from net on vessels");
legend(["Vessel 1", "Vessel 2"]);
xlabel("Time [s]");
ylabel("Force [N]")
grid on;

subplot(3, 2, 3);
plot(timespan, speed_vessel_1, timespan, speed_vessel_2);
title("Speed");
xlabel("Timee [s]");
ylabel("Speed [m/s]")
legend(["Vessel 1", "Vessel 2"]);
grid on;

subplot(3, 2, 4);
plot(timespan, propeller_speed_vessel_1, timespan, propeller_speed_vessel_2);
title("Propeller speed");
legend(["Vessel 1", "Vessel 2"]);
xlabel("Time [s]");
ylabel("Propeller speed [RPM]")
grid on;

subplot(3, 2, 5);
plot(timespan, rudder_angle_vessel_1, timespan, rudder_angle_vessel_2);
title("Rudder angle");
legend(["Vessel 1", "Vessel 2"]);
xlabel("Time [s]");
ylabel("Angle [rad]")
grid on;