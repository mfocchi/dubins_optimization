function dxdt = unicycle_model(x, omega_l, omega_r, params)
    theta = x(3);    

    % ideal wheel speed in rad/s
    omega_wheel_l = omega_l / params.gearbox_ratio; % [rad/s]
    omega_wheel_r = omega_r / params.gearbox_ratio; % [rad/s]
    r = params.sprocket_radius;
    B = params.width; 
    %     % ideal linear and angular velocity
    v_input = r * (omega_wheel_r + omega_wheel_l) / 2.0;
    omega_input = r * (omega_wheel_r - omega_wheel_l) / B;

    dxdt = [v_input*cos(theta); v_input*sin(theta); omega_input]; 
end

