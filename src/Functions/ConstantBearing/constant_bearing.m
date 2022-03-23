function [speed, heading, target_position_offset, inner_turn_output] = constant_bearing(target_position, target_velocity, target_psi, vessel_position, waypoint_index, parameters, waypoints_K)
    
    persistent inner_turn

    if isempty(inner_turn)
        inner_turn = false;
    end

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
    heading = atan2(target_velocity_ned(2), target_velocity_ned(1));
    ff_heading = 0;
    ff_speed = 0;

    if mean(waypoints_K(waypoint_index:waypoint_index + 10)) > 0.0002 && vessel_position(2) < 1000
        inner_turn = true;
    end

    if inner_turn && mean(waypoints_K(waypoint_index: waypoint_index + 2)) < 0.00005
        inner_turn = false;
    end

    if inner_turn
        ff_heading = -sign(heading) * (pi/2) * (1000 * mean(waypoints_K(waypoint_index:waypoint_index + 5)));
        %%ff_speed = 1.5 * (500 * mean(waypoints_K(waypoint_index:waypoint_index + 5)));
    end

    heading = heading + ff_heading;
    speed = speed + ff_speed;

    inner_turn_output = inner_turn;
end
