function [x, y, K] = generate_waypoints()

% scale = 20;
% frequency = 1;
% x = scale * [0 0 100 100 200 200 300 300 400 400 500 500];
% y = scale * [0 400 400 0 0 400 400 0 0 400 400 0];
% time_points = frequency * [0 10 20 30 40 50 60 70 80 90 100 110];
% timespan = 0:1:max(time_points);
% 
% x_s = spline(time_points, x);
% y_s = spline(time_points, y);
% 
% x = ppval(x_s, timespan);
% y = ppval(y_s, timespan);
% 
% d1fx = differentiate(x_s);
% d1fy = differentiate(y_s);
% d2fx = differentiate(d1fx);
% d2fy = differentiate(d1fy);
% 
% d1xi = ppval(d1fx, timespan);
% d1yi = ppval(d1fy, timespan);
% d2xi = ppval(d2fx, timespan);
% d2yi = ppval(d2fy, timespan);
% 
% K = abs(d1xi .* d2yi - d2xi .* d2yi) ./ sqrt(d1xi.^2 + d1yi.^2).^3;

% scale = 1;
% dy = 400;
% dx = 100;
% radius = dx / 2;
% entries = 1;
% 
% x = [];
% y = [];
% 
% for i = 1:entries
%     
%     startx = 0;
%  
%      if i > 1
%          startx = x(end) + dx;
%      end
%     
%     x = [x, startx, startx];
%     y = [y, 0, dy];
%     
%     thetas = linspace(-pi/2, pi/2, 10);
%     ys = radius * cos(thetas) + dy;
%     xs = radius * sin(thetas) + startx + radius;
% 
%     x = [x, xs];
%     y = [y, ys];
% 
%     x = [x, startx + dx, startx + dx];
%     y = [y, dy, 0];
% 
%     thetas = linspace(3*pi/2, pi/2, 10);
%     ys = radius * cos(thetas);
%     xs = radius * sin(thetas) + startx + dx + radius;
%    
%     x = [x, xs];
%     y = [y, ys];
% 
% end
% 
% plot(y, x)

scale = 20;
frequency = 1;
x = scale * [0 0 150 150 300 300 450 450 600 600 750 750];
y = scale * [0 400 400 0 0 400 400 0 0 400 400 0];
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