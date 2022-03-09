function pathplotter(x, y, waypoints, guidance_waypoints)
% pathplotter(x,y) converts NE-positions to longitude and latitude coordinates
% and plots these coordinates over a map using geoplot.
%
% x = position in x-direction (North)         (m)
% y = position in y-direction (East)          (m)
%

% Longitude and Latitude coordinates of the waypoints
% WP0_long = 10.3907; WP0_lat = 63.4367;
% WP1_long = 10.3315; WP1_lat = 63.4556;
% WP2_long = 9.8857;  WP2_lat = 63.4922;
% WP3_long = 9.7813;  WP3_lat = 63.6490;
% WP4_long = 9.6706;  WP4_lat = 63.6866;
% 
% WP_long = [WP0_long WP1_long WP2_long WP3_long WP4_long];
% WP_lat = [WP0_lat WP1_lat WP2_lat WP3_lat WP4_lat];

start_lat = 31.137769;
start_long = 206.744556;

l_0 = deg2rad(start_long); 
mu_0 = deg2rad(start_lat);

figure;
[WP_long, WP_lat, ~] = flat2llh(waypoints(1, :), ...
                                waypoints(2, :), ...
                                zeros(1, length(waypoints(1))), ...
                                l_0, ...
                                mu_0, ...
                                0);

WP_lat = wrapTo180(rad2deg(WP_lat));
WP_long = wrapTo360(rad2deg(WP_long));
geoplot(WP_lat, WP_long, 'x-')
hold on

% 
% [WP_long, WP_lat, ~] = flat2llh(guidance_waypoints(1, :), ...
%                                 guidance_waypoints(2, :), ...
%                                 zeros(1, length(guidance_waypoints(1))), ...
%                                 l_0, ...
%                                 mu_0, ...
%                                 0);
% WP_lat = wrapTo180(rad2deg(WP_lat));
% WP_long = wrapTo360(rad2deg(WP_long));
% geoplot(WP_lat, WP_long, 'yx-')
% hold on



[l, mu, ~] = flat2llh(x, y, zeros(1, length(x)), l_0, mu_0, 0);
l = wrapTo360(rad2deg(l)); 
mu = rad2deg(mu);
geoplot(mu, l, 'r-');
hold off

end

