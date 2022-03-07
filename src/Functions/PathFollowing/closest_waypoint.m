function [point, index] = closest_waypoint(vessel_position, waypoints, start_index)
    
    prev_distance = sqrt((waypoints(1, start_index) - vessel_position(1))^2 + ... 
                         (waypoints(2, start_index) - vessel_position(2))^2);
    
    for i = start_index + 1:length(waypoints)
        x = waypoints(1, i);
        y = waypoints(2, i);
        
        current_distance = sqrt((x - vessel_position(1))^2 + (y - vessel_position(2))^2);

        % Early stopping
        if i > 1 && current_distance > prev_distance
            index = i - 1;
            point = waypoints(:, (i - 1));
            return;
        end

        prev_distance = current_distance;
    end
    
    index = i;
    point = waypoints(:, i);
end

