function [dxdt] = dynamics(t, x, omega_l, omega_r, params) % because we have time invariant system t wont be used

    theta = x(3);    

    % ideal wheel speed in rad/s
    omega_wheel_l = omega_l / params.gearbox_ratio; % [rad/s]
    omega_wheel_r = omega_r / params.gearbox_ratio; % [rad/s]
    r = params.sprocket_radius;
    B = params.width; 
%     % ideal linear and angular velocity
    v_input = r * (omega_wheel_r + omega_wheel_l) / 2.0;
    omega_input = r * (omega_wheel_r - omega_wheel_l) / B;

    % ideal linear and angular velocity
    %v_input,omega_input] =  computeVelocitiesFromTracks(omega_l, omega_r, params);
    switch (params.model)
        case 'UNICYCLE'
    
            dxdt = [v_input*cos(theta); v_input*sin(theta); omega_input];
        
        case 'LONGSLIP'
            turning_radius_input = v_input / omega_input;

            % data collected from a test, the robot was turning left, we are assuming
            % symmetry  

            a0_L = params.slip_fit_coeff.left(1);
            a1_L = params.slip_fit_coeff.left(2);

            a0_R = params.slip_fit_coeff.right(1);
            a1_R = params.slip_fit_coeff.right(2);

            %slip is estimated from turning radius
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
            % estimation of actual wheel speed from slippage
            omega_wheel_act_r = omega_wheel_r * (1-i_R);
            omega_wheel_act_l = omega_wheel_l * (1-i_L);

            % actual linear and angular velocity
            v     = r * (omega_wheel_act_r + omega_wheel_act_l) / 2;
            Omega = r * (omega_wheel_act_r - omega_wheel_act_l) / B;

            dxdt = [v*cos(theta); v*sin(theta); Omega];
       case 'SIDESLIP'
           turning_radius_input = v_input / omega_input;

            % data collected from a test, the robot was turning left, we are assuming
            % symmetry  

            a0_L = params.slip_fit_coeff.left(1);
            a1_L = params.slip_fit_coeff.left(2);

            a0_R = params.slip_fit_coeff.right(1);
            a1_R = params.slip_fit_coeff.right(2);

            a0 = params.side_slip_fit_coeff(1);
            a1 = params.side_slip_fit_coeff(2);

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
            % Slip cannot reach higher values than 1.0
            if(i_L > 1.0)
                i_L = 1.0;
            end
            if(i_R > 1.0)
                i_R = 1.0;
            end
            % actual linear and angular velocity
            v     = r * (omega_wheel_r * (1-i_R) + omega_wheel_l * (1-i_L)) / 2;
            Omega = r * (omega_wheel_r * (1-i_R) - omega_wheel_l * (1-i_L)) / B;

            dxdt = [v*(cos(theta)-sin(theta)*tan(alpha)); v*(sin(theta)+cos(theta)*tan(alpha)); Omega];


        otherwise
            disp('wrong model')
            dxdt = [0;0;0];
    end
end


