function [x, y, K] = generate_waypoints()

x_scale = 20;
y_scale = 20;
frequency = 1;
x = x_scale * [0 0 150 150 300 300 450 450 600 600 750 750];
y = y_scale * [0 400 400 0 0 400 400 0 0 400 400 0];
time_points = frequency * [0 10 32 42 64 74 96 106 128 138 160 170];

timespan = 0:1:max(time_points);

x_s = spline(time_points, x);
y_s = spline(time_points, y);

x = ppval(x_s, timespan);
y = ppval(y_s, timespan);

d1fx = differentiate(x_s);
d1fy = differentiate(y_s);
d2fx = differentiate(d1fx);
d2fy = differentiate(d1fy);

d1xi = ppval(d1fx, timespan);
d1yi = ppval(d1fy, timespan);
d2xi = ppval(d2fx, timespan);
d2yi = ppval(d2fy, timespan);

K = abs(d1xi .* d2yi - d2xi .* d2yi) ./ sqrt(d1xi.^2 + d1yi.^2).^3;

end