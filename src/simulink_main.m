clear;

% Persistent variables in wp_selector, so we clear it
clear wp_selector;
clear guidance_law;
clear heading_control;
clear speed_control;

parameters = construct_parameters();

output = sim("ship", 1000);

x = output.simulation_data.Data(:, 1);
y = output.simulation_data.Data(:, 2);

pathplotter(x, y)