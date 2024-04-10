function resp = OptimCallback(req,resp)
    global params  
    
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
    [omega_l0, omega_r0, t_rough] = getVelocityParamsFromDubin(params, pathSegObj{1}.MotionLengths, omegas);
    
    if strcmp(req.plan_type, "dubins")

    elseif strcmp(req.plan_type, "optim")
    
        solution = optimize_cpp_mex(p0,  pf, omega_l0, omega_r0, t0,  params); 
        plot_solution(solution,p0, pf, params, false);
        resp.des_x = solution.p(1,:);
        resp.des_y = solution.p(2,:);
        resp.des_theta = solution.p(3,:);
    else
        disp("wrong plan type")
    end
    %

end