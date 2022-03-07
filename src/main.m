function main()

clear;

% Persistent variables in wp_selector, so we clear it
clear wp_selector;

parameters = construct_parameters();

%% Waypoints & spline
scale = 100;
frequency = 1;
waypoints.x = scale * [0 0 100 100 200 200 300 300 400 400 500 500];
waypoints.y = scale * [0 375 375 125 125 375 375 125 125 375 375 0];
waypoints.time = frequency * [0 10 32 42 67 77 100 110 135 145 173 193];

timespan = 0:1:max(waypoints.time);
x_s = spline(waypoints.time, waypoints.x, timespan);
y_s = spline(waypoints.time, waypoints.y, timespan);

waypoints = [x_s; y_s];
waypoint_index = 1;

%% Setup
h  = parameters.simulation.step_size;       % sampling time [s]
total_time = 3600*20;
Ns = total_time / h;                        % no. of samples

U_ref = 1.5;                                % desired surge speed (m/s)

% Initial states
eta = [0 0 pi / 2]';
nu  = [0 0 0]';
delta = 0;
n = 0;
Qm = 0;
x = [nu' eta' delta n Qm]';

% Ship parameters
L           = parameters.ship.length;
delta_max   = parameters.rudder.max_angle;

% Path following parameters
look_ahead_distance = parameters.guidance.look_ahead;
kappa               = parameters.guidance.kappa;

% Wind expressed in NED
Vw = 1;
betaVw = deg2rad(135);
rho_a = 1.247;
cy = 0.95;
cn = 0.15;
A_Lw = 10 * L;

% Current
Vc = 1;
betaVc = deg2rad(45);

% Heading reference model & control
psi_ref = 0;
y_int_dot = 0;
psi_ref_x = [0; 0; 0];
e_psi_int = 0;
y_int = 0;


% Speed control
e_u_int = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simdata = zeros(Ns+1,14);       % table of simulation data

next_waypoints = [];

for i=1:Ns+1
    
    t = (i-1) * h;              % time (s)
    
    %% Current disturbance

    uc = Vc * cos(betaVc - x(6)); 
    vc = Vc * sin(betaVc - x(6)); 
    % nu_c = [uc vc 0]';
    nu_c = [0 0 0]';

    %% Wind disturbance

    % Wind velocity in world frame
    uw = Vw * cos(betaVw - x(6)); 
    vw = Vw * sin(betaVw - x(6)); 
    
    % These are relative to the boat's current velocity
    u_rw = x(1) - uw;
    v_rw = x(2) - vw;
    V_rw = sqrt(u_rw^2 + v_rw^2); 
    gamma_rw = -atan2(v_rw, u_rw);
    
    Ywind = 0.5 * rho_a * V_rw^2 * cy * sin(gamma_rw) * A_Lw;
    Nwind = 0.5 * rho_a * V_rw^2 * cn * sin(2 * gamma_rw) * A_Lw * L;

    % tau_wind = [0 Ywind Nwind]';
    tau_wind = [0 0 0]';
        
    %% Guidance law
    
    % [xk1, yk1, xk, yk, ~] = wp_selector(x(4), x(5), parameters.ship.length); 
    % [e_y, pi_p] = cross_track_error(xk1, yk1, xk, yk, x(4), x(5));
    % [psi_desired, y_int_dot] = ilos_guidance(e_y, pi_p, y_int, look_ahead_distance, kappa);
    % psi_ref = psi_desired;

    [current_waypoint, waypoint_index] = closest_waypoint(x(4:5), waypoints, waypoint_index);

    if waypoint_index < length(waypoints)
        next_waypoint = waypoints(:, waypoint_index + 1);
    else
        next_waypoint = waypoints(:, waypoint_index);
    end
    
    % [next_waypoint, found] = find_farthest_point_spanned_by_circle(2000, waypoints, waypoint_index);

    found = true;
    if found
        [e_y, pi_p] = cross_track_error(next_waypoint(1), ...
                                        next_waypoint(2), ...
                                        current_waypoint(1), ...
                                        current_waypoint(2), ...
                                        x(4), ...
                                        x(5));
        [psi_desired, y_int_dot] = ilos_guidance(e_y, pi_p, y_int, look_ahead_distance, kappa);
        psi_ref = psi_desired;
        
        next_waypoints = [next_waypoints; next_waypoint'];
    end

    %% Heading control

    % The derivative coming from the reference model is the state
    % derivative of the reference model
    psi_ref_x_dot = heading_reference_model(psi_ref_x, psi_ref);

    % We can the extract the desired state for heading (psi)
    % and heading rate (r) from the reference model
    psi_desired = psi_ref_x(1);
    r_desired = psi_ref_x(2);

    % Control law
    e_psi = ssa(x(6) - psi_desired);
    e_r = ssa(x(3) - r_desired);

    % Now we retrieve the rudder angle command from the heading PID
    delta_c = heading_pid(e_psi, e_r, e_psi_int, parameters.nomoto); 

    % Anti-integrator windup
    if abs(delta_c) > delta_max
       delta_c = delta_max * sign(delta_c);
       e_psi_int = e_psi_int - h * e_psi;
       y_int = y_int - h * y_int_dot;
    end

    %% Speed control
    u_desired = U_ref;
    e_u = x(1) - u_desired;

    % Here n_c is the rps of the motor (n) commanded 
    n_c = speed_pid(u_desired, e_u, e_u_int, parameters);

    %% Ship dynamics
    u = [delta_c n_c]';
    [xdot, u] = ship_model(x, u, nu_c, tau_wind, parameters);
    
    %% Store simulation data in a table (for testing)
    simdata(i,:) = [t x(1:3)' x(4:6)' x(7) x(8) u(1) u(2)...
                    u_desired psi_desired r_desired];     
 
    % Euler integration
    
    % Make integration in batch to save some time
    s     = [x; psi_ref_x; e_psi_int; e_u_int; y_int];
    s_dot = [xdot; psi_ref_x_dot; e_psi; e_u; y_int_dot];
    
    s = euler2(s_dot, s, h);

    x = s(1:9);
    psi_ref_x = s(10:12);
    e_psi_int = s(13);
    e_u_int = s(14);
    y_int = s(15);
% 
%     x = euler2(xdot, x, h);
%     psi_ref_x = euler2(psi_ref_x_dot, psi_ref_x, h);
%     e_psi_int = euler2(e_psi, e_psi_int, h);
%     e_u_int = euler2(e_u, e_u_int, h);
%     y_int = euler2(y_int_dot, y_int, h);

end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t       = simdata(:,1);                 % s
u       = simdata(:,2);                 % m/s
v       = simdata(:,3);                 % m/s
r       = (180/pi) * simdata(:,4);      % deg/s
x       = simdata(:,5);                 % m
y       = simdata(:,6);                 % m
psi     = (180/pi) * simdata(:,7);      % deg
delta   = (180/pi) * simdata(:,8);      % deg
n       = 60 * simdata(:,9);            % rpm
delta_c = (180/pi) * simdata(:,10);     % deg
n_c     = 60 * simdata(:,11);           % rpm
u_d     = simdata(:,12);                % m/s
psi_d   = (180/pi) * simdata(:,13);     % deg
r_d     = (180/pi) * simdata(:,14);     % deg/s


figure(1)
figure(gcf)
subplot(311)
plot(y,x,'linewidth',2); axis('equal')
title('North-East positions (m)'); xlabel('(m)'); ylabel('(m)'); 
subplot(312)
plot(t,psi,t,psi_d,'linewidth',2);
title('Actual and desired yaw angles (deg)'); xlabel('time (s)');
legend(["Actual", "Desired"]);
subplot(313)
plot(t,r,t,r_d,'linewidth',2);
title('Actual and desired yaw rates (deg/s)'); xlabel('time (s)');
legend(["Actual", "Desired"]);


figure(2)
figure(gcf)
subplot(311)
plot(t,u,t,u_d,'linewidth',2);
title('Actual and desired surge velocities (m/s)'); xlabel('time (s)');
legend(["Actual", "Desired"]);
subplot(312)
plot(t,n,t,n_c,'linewidth',2);
title('Actual and commanded propeller speed (rpm)'); xlabel('time (s)');
legend(["Actual", "Desired"]);
subplot(313)
plot(t,delta,t,delta_c,'linewidth',2);
title('Actual and commanded rudder angles (deg)'); xlabel('time (s)');
legend(["Actual", "Desired"]);

pathplotter(x, y, [x_s; y_s], next_waypoints');
end