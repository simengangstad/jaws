function pathplotter(x,y)
% pathplotter(x,y) converts NE-positions to longitude and latitude coordinates
% and plots these coordinates over a map using geoplot.
%
% x = position in x-direction (North)         (m)
% y = position in y-direction (East)          (m)
%

% Longitude and Latitude coordinates of the waypoints
WP0_long = 10.3907; WP0_lat = 63.4367;
WP1_long = 10.3315; WP1_lat = 63.4556;
WP2_long = 9.8857;  WP2_lat = 63.4922;
WP3_long = 9.7813;  WP3_lat = 63.6490;
WP4_long = 9.6706;  WP4_lat = 63.6866;

WP_long = [WP0_long WP1_long WP2_long WP3_long WP4_long];
WP_lat = [WP0_lat WP1_lat WP2_lat WP3_lat WP4_lat];

l_0 = deg2rad(WP0_long); mu_0 = deg2rad(WP0_lat); h_ref = 0;

figure;
geoplot(WP_lat,WP_long,'x-')
[l,mu,~] = flat2llh(x,y,zeros(1,length(x)),l_0,mu_0,h_ref);
l = rad2deg(l); mu = rad2deg(mu);
hold on
fig = geoplot(mu,l,'r-');
% saveas(fig, "Figures/oppg5c_map", "epsc");
hold off

end

