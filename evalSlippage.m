function [R, i_L, i_R] = evalSlippage(theta, omega_l, omega_r, params)
        omega_wheel_l = omega_l / params.gearbox_ratio; % [rad/s]
        omega_wheel_r = omega_r / params.gearbox_ratio; % [rad/s]
   
        r = params.sprocket_radius;
        B = params.width;

        v_input = r * (omega_wheel_r + omega_wheel_l) / 2.0;
        omega_input = r * (omega_wheel_r - omega_wheel_l) / B;
        turning_radius_input = v_input ./ omega_input;
        a0_L = params.slip_fit_coeff.left(1);
        a1_L = params.slip_fit_coeff.left(2);

        a0_R = params.slip_fit_coeff.right(1);
        a1_R = params.slip_fit_coeff.right(2);

        R = abs(turning_radius_input);
        for i=1:length(turning_radius_input)
            if(turning_radius_input(i) > 0.0) % turning left
                i_L(i) = a0_L / (R(i) + a1_L);
                i_R(i) = a0_R / (R(i) + a1_R);
            else % turning right
                % inverting the slips with respect to test results
                i_R(i) = a0_L / (R(i) + a1_L);
                i_L(i) = a0_R / (R(i) + a1_R);
            end
            % Slip cannot reach higher values than 1.0
            if(i_L(i) > 1.0)
                i_L(i) = 1.0;
            end
            if(i_R(i) > 1.0)
                i_R(i) = 1.0;
            end
        end
end