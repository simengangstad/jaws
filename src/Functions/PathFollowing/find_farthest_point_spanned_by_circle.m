function [farthest_point, found] = find_farthest_point_spanned_by_circle(radius, waypoints, start_index)
    
    found = false;
    farthest_point = [];

    tangential_point = waypoints(:, start_index);
    
    for i = start_index + 1:length(waypoints)
        x = waypoints(1, i);
        y = waypoints(2, i);

        distance = sqrt((x - tangential_point(1))^2 + (y - tangential_point(2))^2);

        % Set the farthest point to the one furthest away from us, but
        % within the circle
        if distance < radius
            farthest_point = [x, y];
            found = true;
        else
            break;
        end
    end
end

