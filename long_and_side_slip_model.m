function dxdt = long_and_side_slip_model(x, wheel_l, wheel_r, params)
    %omega_l/r are the unicycle wheel speed
    theta = x(3);    
    % ideal wheel speed in rad/s
    r = params.sprocket_radius;
    B = params.width; 
    %     % ideal linear and angular velocity
    v_input = r * (wheel_l + wheel_l) / 2.0;
    omega_input = r * (wheel_r - wheel_l) / B;
    
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

    %long slip
    %compute track velocity from encoder
    v_enc_l = r*wheel_l;
    v_enc_r = r*wheel_r;

    %estimate beta_inner, beta_outer from turning radius
    if(turning_radius_input >= 0.0)% turning left, positive radius, left wheel is inner right wheel is outer
        beta_l = params.beta_slip_inner_coefficients_left(1)*exp(params.beta_slip_inner_coefficients_left(2)*turning_radius_input);
        % the inner is accelerated the outer slowed down
        beta_r = params.beta_slip_outer_coefficients_left(1)*exp(params.beta_slip_outer_coefficients_left(2)*turning_radius_input);
    else % turning right, negative radius, left wheel is outer right is inner
        beta_r = params.beta_slip_inner_coefficients_right(1)*exp(params.beta_slip_inner_coefficients_right(2)*turning_radius_input);     
        beta_l =  params.beta_slip_outer_coefficients_right(1)*exp(params.beta_slip_outer_coefficients_right(2)*turning_radius_input);
    end

    v_enc_l=v_enc_l+beta_l;
    v_enc_r=v_enc_r+beta_r;

    wheel_l_s = 1/r * v_enc_l;
    wheel_r_s = 1/r * v_enc_r;

    % actual linear and angular velocity
    v_input_s     = r * (wheel_r_s + wheel_l_s) / 2;
    omega_input_s = r * (wheel_r_s - wheel_l_s) / B;
         
    dxdt = [v_input_s*(cos(theta)-sin(theta)*tan(alpha)); v_input_s*(sin(theta)+cos(theta)*tan(alpha)); omega_input_s];
end

