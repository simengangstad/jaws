clear;

parameters = construct_parameters();

%% Waypoints & spline

[x, y, K] = generate_waypoints();
waypoints = [x; y];
waypoint_index = 1;
next_waypoints = [];

%% Setup
h  = parameters.simulation.step_size;       % sampling time [s]
total_time = 3 * 3600;
Ns = total_time / h;                        % no. of samples

U_ref = 1.5;                                % desired surge speed (m/s)

% Initial states
eta = [0 0 pi / 2 + 0.2]';
nu  = [0 0 0]';
delta = 0;
n = 0;
Qm = 0;
x = [nu' eta' delta n Qm]';

% Path following parameters
look_ahead_distance = parameters.guidance.look_ahead;
kappa               = parameters.guidance.kappa;

% Heading reference model & control
psi_ref_x = [0; 0; 0];
e_psi_int = 0;
y_int = 0;

% Speed control
e_u_int = 0;

% Wind expressed in NED
Vw = 1;
betaVw = deg2rad(135);
rho_a = 1.247;
cy = 0.95;
cn = 0.15;
A_Lw = 10 * parameters.ship.length;

% Current
Vc = 0.2;
betaVc = deg2rad(45);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simdata = zeros(Ns+1, length(x) + 5);       % table of simulation data

for i=1:Ns+1
    
    t = (i-1) * h;              % time (s)

    %% Current disturbance

    uc = Vc * cos(betaVc - x(6)); 
    vc = Vc * sin(betaVc - x(6)); 
    nu_c = [uc vc 0]';

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
    Nwind = 0.5 * rho_a * V_rw^2 * cn * sin(2 * gamma_rw) * A_Lw * parameters.ship.length;

    tau_wind = [0 Ywind Nwind]';


    %% Guidance law
    [current_waypoint, waypoint_index] = closest_waypoint(x(4:5), waypoints, waypoint_index);
    
    radius = 1000 * (1 - 175 * K(waypoint_index));

    [next_waypoint, found] = find_farthest_point_spanned_by_circle(radius, waypoints, waypoint_index);

    if ~found
        next_waypoint = waypoints(:, waypoint_index + 1)';
    end
    next_waypoint = waypoints(:, waypoint_index + 1)';

    [e_y, pi_p] = cross_track_error(next_waypoint(1), ...
                                    next_waypoint(2), ...
                                    current_waypoint(1), ...
                                    current_waypoint(2), ...
                                    x(4), ...
                                    x(5));

    look_ahead = look_ahead_distance * (1 - 150 * K(waypoint_index));

    [psi_desired, y_int_dot] = ilos_guidance(e_y, pi_p, y_int, look_ahead, kappa);
    psi_ref = psi_desired;
    
    next_waypoints = [next_waypoints; next_waypoint];

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
    if abs(delta_c) > parameters.rudder.max_angle
       delta_c = parameters.rudder.max_angle * sign(delta_c);
       e_psi_int = e_psi_int - h * e_psi;
    end

    %% Speed control
    u_desired = U_ref + 300 * K(waypoint_index);
    e_u = x(1) - u_desired;

    % Here n_c is the rps of the motor (n) commanded 
    n_c = speed_pid(u_desired, e_u, e_u_int, parameters);

    %% Ship dynamics
    u = [delta_c n_c]';
    [x_dot, u] = supply_model(x, u, nu_c, tau_wind, parameters);
    
    % Store simulation data in a table (for testing)
    simdata(i,:) = [t x(1:3)' x(4:6)' x(7) x(8) u(1) u(2)...
                    u_desired psi_desired r_desired];     
 
    %% Euler integration
    % Make integration in batch to save some time
    s     = [x; psi_ref_x; e_psi_int; e_u_int; y_int];
    s_dot = [x_dot; psi_ref_x_dot; e_psi; e_u; y_int_dot];
    
    s = euler2(s_dot, s, h);

    x = s(1:9);
    psi_ref_x = s(10:12);
    e_psi_int = s(13);
    e_u_int = s(14);
    y_int = s(15);

    if mod(i, 3600 / h) == 0
        hour = i * h / 3600;
        fprintf("Done simulating %i/%.1f hours\n", hour, total_time / 3600.0);
    end

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

pathplotter(x, y, waypoints, next_waypoints');