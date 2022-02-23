function [xk1, yk1, xk, yk, last] = wp_selector(x, y, L) 
    last = 0;
    persistent first k WP_ R_k_1

    if isempty(first)
        WP = load('WP.mat');
        first = 0;
        k = 1;
        WP_ = [WP.WP(1,:), WP.WP(2,:)];
        R_k_1 = 4 * L;
    end

    n = length(WP_) / 2;            % Number of waypoints
    wpt.x = WP_(1:n);               % North coordinates
    wpt.y = WP_(n+1:2*n);           % East coordinates

    % Choose the correct waypoints
    xk  = wpt.x(k);
    xk1 = wpt.x(k+1);
    yk  = wpt.y(k);
    yk1 = wpt.y(k+1);

    % Update waypoints
    if (xk1 - x)^2 + (yk1 - y)^2 <= R_k_1^2 
        
        if k < n - 1
            k = k + 1;
        else
            last = 1;
        end 
    end
end