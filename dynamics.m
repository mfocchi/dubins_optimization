function [dxdt] = dynamics(t, x, omega_l, omega_r, params) % because we have time invariant system t wont be used
    switch (params.model)
        case 'UNICYCLE'
            dxdt = unicycle_model(x, omega_l, omega_r, params);        
        case 'LONGSLIP'
            dxdt = long_slip_model(x, omega_l, omega_r, params);
        case 'LONGSLIP_SIDESLIP_LOCKED_WHEEL'
            dxdt = long_and_side_slip_locked_wheel_model(x, omega_l, omega_r, params);
        case 'LONGSLIP_SIDESLIP'
            dxdt = long_and_side_slip_model(x, omega_l, omega_r, params);
        otherwise
            disp('wrong model')
            dxdt = [0;0;0];
    end
end


