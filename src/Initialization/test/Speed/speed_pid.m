function nd = speed_pid(Ud, e_u, e_int)
    % Input:
    %
    % Ud: Desired velocity
    % e_u: Error in velocity
    % e_int: Integrated error
    % 
    % Output:
    % 
    % nd: Desired propeller speed (rps)

    % Propeller parameters
    Dia = 3.3;                  % Propeller diameter (m)
    rho = 1025;                 % Density of sea water (m/s^3)
    t_thr = 0.05;
    KT = 0.6367;
    m = 17.0677e6; % mass (kg) 
    Xudot = -8.9830e5;
    T1 = 20;
    Xu = -(m - Xudot) / T1;
    Td = -Xu * Ud / (1 - t_thr);                   % Desired thrust

    % PI controller gains
    Kp = 0.5;
    Ki = 0.05;
    delta_u = -Kp * e_u - Ki * e_int;

    % Desired propeller speed (rps)
    nd = sign(Td) * sqrt(Td / (rho * Dia^4 * KT)) + delta_u; 
end