function dxdt = tracked_robot_model(x, u, param)

    omega_wheel_l = u(1) / param.gearbox_ratio; % [rad/s]
    omega_wheel_r = u(2) / param.gearbox_ratio; % [rad/s]
    theta = x(3);

    r = param.sprocket_radius;
    B = param.width;

    v_input = r * (omega_wheel_r + omega_wheel_l) / 2.0;
    omega_input = r * (omega_wheel_r - omega_wheel_l) / B;
    turning_radius_input = v_input / omega_input;

    % data collected from a test, the robot was turning left, we are assuming
    % symmetry  

    a0_L = param.slip_fit_coeff.left(1);
    a1_L = param.slip_fit_coeff.left(2);

    a0_R = param.slip_fit_coeff.right(1);
    a1_R = param.slip_fit_coeff.right(2);

    R = abs(turning_radius_input);
    if(turning_radius_input > 0.0) % turning left
        i_L = a0_L / (R + a1_L);
        i_R = a0_R / (R + a1_R);
    else % turning right
        % inverting the slips with respect to test results
        i_R = a0_L / (R + a1_L);
        i_L = a0_R / (R + a1_R);
    end
    % Slip cannot reach higher values than 1.0
    if(i_L > 1.0)
        i_L = 1.0;
    end
    if(i_R > 1.0)
        i_R = 1.0;
    end
      if(i_L <-1000)
        i_L = 1000;
    end
    if(i_R < -1000)
        i_R = -1000;
    end
    

    v     = r * (omega_wheel_r * (1-i_R) + omega_wheel_l * (1-i_L)) / 2;
    Omega = r * (omega_wheel_r * (1-i_R) - omega_wheel_l * (1-i_L)) / B;
    
    dxdt = [v*cos(theta), v*sin(theta), Omega];
end

