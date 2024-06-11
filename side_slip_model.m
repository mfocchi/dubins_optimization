function dxdt = side_slip_model(x, omega_l, omega_r, params)
    %omega_l/r are the unicycle wheel speed
    theta = x(3);    
    % ideal wheel speed in rad/s
    r = params.sprocket_radius;
    B = params.width; 
    %     % ideal linear and angular velocity
    v_input = r * (omega_r + omega_l) / 2.0;
    omega_input = r * (omega_r - omega_l) / B;

    if omega_input == 0 &&  v_input~=0
        turning_radius_input =  1e08*sign(v_input);
    elseif omega_input == 0 &&  v_input==0
        
        turning_radius_input = 1e8;
    else
         turning_radius_input = v_input / omega_input;
    end
    
    % side slip        
    if(turning_radius_input > 0.0) % turning left 
        alpha = params.side_slip_angle_coefficients_left(1)*exp(params.side_slip_angle_coefficients_left(2)*turning_radius_input);
    else % turning right
        % inverting the slips with respect to test results   
        alpha = params.side_slip_angle_coefficients_right(1)*exp(params.side_slip_angle_coefficients_right(2)*turning_radius_input);
    end
       
    dxdt = [v_input*(cos(theta)-sin(theta)*tan(alpha)); v_input*(sin(theta)+cos(theta)*tan(alpha)); omega_input];
end

