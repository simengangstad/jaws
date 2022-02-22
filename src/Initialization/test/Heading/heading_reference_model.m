function ref_x_dot = heading_reference_model(ref_x, ref)
    % Input:
    %
    % ref_x: State of the reference (3x1). This is a second order model so
    % we keep track of three states, where the first one is the reference
    % model's angle reference.
    % ref: The desired reference
    % 
    % Output:
    % 
    % ref_x_dot: Derivative of the state of the reference

    wn_ref = 0.03;
    Ad = [0 1 0; 
          0 0 1; 
          -wn_ref^3 -3 * wn_ref^2 -3 * wn_ref]; 
    
    Bd = [0 0 wn_ref^3]';
    
    ref_x_dot = Ad * ref_x + Bd * ref;
end


