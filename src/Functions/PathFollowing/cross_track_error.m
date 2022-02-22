function [e_y, pi_p] = cross_track_error(x2, y2, x1, y1, x, y)
    % Input:
    % (x2, y2): Next waypoint
    % (x1, y1): Previous waypoint
    % (x, y): Current position
    %
    % Output
    % e_y: cross-track error (m) expreesed in NED
    % pi_p: path-tangential angle with respect to the North axis 
    
    pi_p = atan2(y2 - y1, x2 - x1);
    e_y = -(x - x1) * sin(pi_p) + (y - y1) * cos(pi_p);
end