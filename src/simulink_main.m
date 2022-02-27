clear;

parameters = construct_parameters();

output = sim('ship', 6000);

x = output.simulation_data.Data(:, 1);
y = output.simulation_data.Data(:, 2);

pathplotter(x, y)