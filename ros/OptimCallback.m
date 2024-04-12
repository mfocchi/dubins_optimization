function resp = OptimCallback(req,resp)
    global params  
    clc
    p0(1) = req.x0;
    p0(2) = req.y0;
    p0(3) = req.theta0;
    
    pf(1) = req.xf;
    pf(2) = req.yf;
    pf(3) = req.thetaf;
    
    %make them columns
    p0 = p0(:);
    pf = pf(:);
    
    dubConnObj = dubinsConnection;
    curvature_max = params.omega_max/params.v_max;
    dubConnObj.MinTurningRadius = 1/curvature_max;
    disp(dubConnObj.MinTurningRadius);
    [pathSegObj, pathCosts] = connect(dubConnObj,p0',pf');
    %show(pathSegObj{1})
    % get total time
    t0 = sum(pathSegObj{1}.MotionLengths)/params.v_max;
  
    % compute the omega from dubin
    omegas = get_omega_from_dubins(pathSegObj{1}, params.v_max, 1/curvature_max);
    %map to wheel omega
    [omega_l0, omega_r0, t_rough] = getWheelVelocityParamsFromDubin(params, pathSegObj{1}.MotionLengths, omegas);
    
    if strcmp(req.plan_type, "dubins")
        %generate inputs from dubins om a fome grid (dt)
        [omega_l_fine, omega_r_fine, ] = getWheelVelocityParamsFromDubin(params, pathSegObj{1}.MotionLengths, omegas, params.dt);
        %integrate dubin on a fine grid (dt)
        params.int_steps = 0;
        params.model = 'UNICYCLE'; %need to set unicycle!!!
        %integrate the fine grid omegas
        [states, t] = computeRollout(p0, 0,params.dt, length(omega_l_fine), omega_l_fine, omega_r_fine, params);
        resp.des_x = states(1,:);
        resp.des_y = states(2,:);
        resp.des_theta = states(3,:);
        [v_input,omega_input] = computeVelocitiesFromTracks(omega_l_fine, omega_r_fine, params)
        resp.des_v = v_input;
        resp.des_omega = omega_input;
        resp.dt = params.dt;
        

    elseif strcmp(req.plan_type, "optim")    
        solution = optimize_cpp_mex(p0,  pf, omega_l0, omega_r0, t0,  params); 
        plot_solution(solution,p0, pf, params, false);
        resp.des_x = solution.p_fine(1,:);
        resp.des_y = solution.p_fine(2,:);
        resp.des_theta = solution.p_fine(3,:);
        resp.des_v = solution.v_input_fine;
        resp.des_omega = solution.omega_input_fine;
        resp.dt = params.dt;


    else
        disp("wrong plan type")
    end
    %
    req.plan_type
    resp.des_x(end-10:end)
    resp.des_y(end-10:end)
    resp.des_theta(end-10:end)
    resp.des_v(end-10:end)
    resp.des_omega(end-10:end)
    resp.dt
end