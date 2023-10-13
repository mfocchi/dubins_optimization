function dxdt = long_and_side_slip_model(x, omega_l, omega_r, params)
    theta = x(3);    
    % ideal wheel speed in rad/s
    omega_wheel_l = omega_l; % [rad/s]
    omega_wheel_r = omega_r; % [rad/s]
    r = params.sprocket_radius;
    B = params.width; 
    %     % ideal linear and angular velocity
    v_input = r * (omega_wheel_r + omega_wheel_l) / 2.0;
    omega_input = r * (omega_wheel_r - omega_wheel_l) / B;
    
    a0_L = params.slip_fit_coeff.left(1);
    a1_L = params.slip_fit_coeff.left(2);
    a0_R = params.slip_fit_coeff.right(1);
    a1_R = params.slip_fit_coeff.right(2);
    a0 = params.side_slip_fit_coeff(1);
    a1 = params.side_slip_fit_coeff(2);

    turning_radius_input = v_input / omega_input;
    R = abs(turning_radius_input);
    
    i_inner = a0_L / (R + a1_L);
    i_outer = a0_R / (R + a1_R);
    side_slip = a0 * exp(R * a1);
        
    if(turning_radius_input > 0.0) % turning left
        i_L = i_inner;
        i_R = i_outer;
        alpha = side_slip;
    else % turning right
        % inverting the slips with respect to test results
        i_R = i_inner;
        i_L = i_outer;
        alpha = -side_slip;
    end
    
    if(i_L > 1.0) % full slip
        v_wheel_l = 0.0;
    elseif(i_L < params.slip_fit_coeff.min_value) % locked track
        v_wheel_l = 0.0;
    else % nominal condition
        v_wheel_l = r * omega_wheel_l * (1-i_L);
    end
    if(i_R > 1.0) % full slip
        v_wheel_r = 0.0;
    elseif(i_R < params.slip_fit_coeff.min_value) % locked track
        v_wheel_r = 0.0;
    else % nominal condition
        v_wheel_r = r * omega_wheel_r * (1-i_R);
    end
    % actual linear and angular velocity
    v     = (v_wheel_r + v_wheel_l) / 2;
    Omega = (v_wheel_r - v_wheel_l) / B;
    
    dxdt = [v*(cos(theta)-sin(theta)*tan(alpha)); v*(sin(theta)+cos(theta)*tan(alpha)); Omega];
end

