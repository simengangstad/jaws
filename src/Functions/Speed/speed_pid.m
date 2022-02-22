function nd = speed_pid(Ud, e_u, e_int, parameters)
    % Input:
    %
    % Ud: Desired velocity
    % e_u: Error in velocity
    % e_int: Integrated error
    % parameters: Parameters related to the ship
    % 
    % Output:
    % 
    % nd: Desired propeller speed (rps)

    w       = parameters.propeller.wake_fraction_number;
    KT      = parameters.propeller.coefficients.KT;            
    Dia     = parameters.propeller.diameter;           
    rho     = parameters.propeller.density_of_water;
    m       = parameters.ship.mass;
    Xudot   = parameters.added_mass.Xudot;
    T1      = parameters.natural_periods.surge;
    
    Xu      = -(m - Xudot) / T1;
    Td      = -Xu * Ud / (1 - w);                   % Desired thrust

    % PI controller gains
    Kp = 0.5;
    Ki = 0.05;
    delta_u = -Kp * e_u - Ki * e_int;

    % Desired propeller speed (rps)
    nd = sign(Td) * sqrt(Td / (rho * Dia^4 * KT)) + delta_u; 
end