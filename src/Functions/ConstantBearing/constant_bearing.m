function [speed, heading] = constant_bearing(target_position, target_velocity, target_psi, vessel_position, parameters)

    span = 500;

    target_velocity_ned = Rzyx(0, 0, target_psi) * [target_velocity(1); target_velocity(2); 0];
    target_position_offset = target_position;

    if norm(target_velocity_ned) > 1e-3
        offset = span * Rzyx(0, 0, pi / 2) * target_velocity_ned / norm(target_velocity_ned);
        offset = offset(1:2);
        target_position_offset = target_position + offset;
    end

    position_error = vessel_position - target_position_offset;

    kappa = parameters.CB.U_max * norm(position_error) / sqrt(position_error' * position_error + parameters.CB.Delta^2);

    target_velocity_ned = target_velocity_ned(1:2);

    if abs(position_error) < 1e-6
        v = [0; 0];
    else
        v = target_velocity_ned - kappa * position_error / norm(position_error);
    end

    speed = norm(v);
    heading = ssa(atan2(target_velocity_ned(2), target_velocity_ned(1)));
end
