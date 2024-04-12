function dxdt = long_and_side_slip_model(x, omega_l, omega_r, params)
    %omega_l/r are the unicycle wheel speed
    theta = x(3);    
    % ideal wheel speed in rad/s
    r = params.sprocket_radius;
    B = params.width; 
    %     % ideal linear and angular velocity
    v_input = r * (omega_r + omega_l) / 2.0;
    omega_input = r * (omega_r - omega_l) / B;
    
    a0_L = params.slip_fit_coeff.left(1);
    a1_L = params.slip_fit_coeff.left(2);
    a0_R = params.slip_fit_coeff.right(1);
    a1_R = params.slip_fit_coeff.right(2);
    a0 = params.side_slip_fit_coeff(1);
    a1 = params.side_slip_fit_coeff(2);

    turning_radius_input = v_input / omega_input;
    R = abs(turning_radius_input);

    side_slip = a0 * exp(R * a1);
        
    if(turning_radius_input > 0.0) % turning left
 
        alpha = side_slip;
    else % turning right
        % inverting the slips with respect to test results   
        alpha = -side_slip;
    end
    
    %%TODO ADD LONG SLIP
    % actual linear and angular velocity
    % v     = (v_wheel_r + v_wheel_l) / 2;
    % Omega = (v_wheel_r - v_wheel_l) / B;
    
    dxdt = [v_input*(cos(theta)-sin(theta)*tan(alpha)); v_input*(sin(theta)+cos(theta)*tan(alpha)); omega_input];
end

