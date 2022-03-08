function delta_c = heading_pid(e_psi, e_r, e_int, nomoto_parameters)
    % Input:
    %
    % e_psi: error in heading
    % e_r: error in heading rate
    % e_int: integrated error
    % nomoto_parameters: Parameters from ship model specifying the nomoto
    % model
    %
    % Output:
    % 
    % delta_c: Rudder angle command (rad)
    
    % Nomoto time and gain constant
    Tn = nomoto_parameters.Tn; 
    Kn = nomoto_parameters.Kn;
    wb = nomoto_parameters.wb;

    zeta = 0.8;
    wn = 1 / sqrt(1 - 2 * zeta^2 + sqrt(4 * zeta^4 - 4 * zeta^2 + 2)) * wb;

    % PID gains based on first-order Nomoto model 
    Kp = wn^2 * Tn / Kn;
    Kd = (2 * zeta * wn * Tn - 1) / Kn;
    Ki = Kp * wn / 10;

    delta_c = -Kp * e_psi - Kd * e_r - Ki * e_int; 
end

