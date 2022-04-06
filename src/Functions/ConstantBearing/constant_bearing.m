function [speed, heading, target_position_offset] = constant_bearing(target_position, target_velocity, target_psi, vessel_position, waypoint_index, rudder_angle_commanded, parameters, waypoints_K, waypoints)

    persistent y_int
    
    if isempty(y_int)
        y_int = 0;
    end

    span = 500;

    current_waypoint = waypoints(:, waypoint_index)';
    next_waypoint = waypoints(:, waypoint_index + 1)';

    offset = next_waypoint - current_waypoint;
    offset = [offset(1), offset(2), 0]';
    offset = span * Rzyx(0, 0, -pi / 2) * offset / norm(offset);
    
    offset = offset(1:2)';
    current_waypoint = current_waypoint + offset;
    next_waypoint = next_waypoint + offset;

    [e_y, pi_p] = cross_track_error(next_waypoint(1), ...
                                    next_waypoint(2), ...
                                    current_waypoint(1), ...
                                    current_waypoint(2), ...
                                    vessel_position(1), ...
                                    vessel_position(2));
    
    look_ahead = parameters.guidance.look_ahead * (1 - 450 * waypoints_K(waypoint_index));

    [psi_desired, y_int_dot] = ilos_guidance(e_y, pi_p, y_int, look_ahead, parameters.guidance.kappa);
    heading = psi_desired;

    % Anti-integrator windup
    if abs(rudder_angle_commanded) <= parameters.rudder.max_angle
        y_int = euler2(y_int_dot, y_int, parameters.simulation.step_size);
    end

    target_velocity_ned = Rzyx(0, 0, target_psi) * [target_velocity(1); target_velocity(2); 0];
    target_position_offset = target_position;

    if norm(target_velocity_ned) > 1e-1
        offset = span * Rzyx(0, 0, -pi / 2) * target_velocity_ned / norm(target_velocity_ned);
        offset = offset(1:2);
        target_position_offset = target_position + offset;
    else
        target_position_offset = current_waypoint';
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
end
