function [psi_desired, y_int_dot] = ilos_guidance(e_y, pi_p, y_int, Delta, kappa)

    Kp = 1 / Delta;
    Ki = kappa * Kp;

    psi_desired = pi_p - atan(Kp * e_y + Ki * y_int);
    y_int_dot = Delta * e_y / ( Delta^2 + (e_y + kappa * y_int)^2 );
end