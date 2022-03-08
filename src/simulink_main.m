clear;

parameters = construct_parameters();
[waypoints_x, waypoints_y, waypoints_K] = generate_waypoints();
waypoints = [waypoints_x; waypoints_y];

output = sim('ship', 3 * 3600);

x = output.simulation_data.Data(:, 1);
y = output.simulation_data.Data(:, 2);

pathplotter(x, y, waypoints, [])