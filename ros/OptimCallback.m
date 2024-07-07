function resp = OptimCallback(req,resp, type_of_ros)

    if nargin<3
        type_of_ros = 'ros2'
    end

    global params  
    clc

    if strcmp(type_of_ros, 'ros2')
        p0(1) = req.x0;
        p0(2) = req.y0;
        p0(3) = req.theta0;
        
        pf(1) = req.xf;
        pf(2) = req.yf;
        pf(3) = req.thetaf;
    elseif strcmp(type_of_ros, 'ros1')
        p0(1) = req.X0;
        p0(2) = req.Y0;
        p0(3) = req.Theta0;
        
        pf(1) = req.Xf;
        pf(2) = req.Yf;
        pf(3) = req.Thetaf;
    else
        disp('wrong ros version')
    end
    %make them columns
    p0 = p0(:);
    pf = pf(:);
    
    fprintf(2,"NEW TARGET:%f %f %f \n", pf);

    %check if omega is feasible
    [feas_omega_min, feas_omega_max] = computeFeasibleOmega(params.v_max, params);
    if (params.omega_max)>feas_omega_max
        fprintf(2, "OMEGA IN DUBINS IS BEYOND THE LIMITS setting to:  %f\n",feas_omega_max );
        params.omega_max = feas_omega_max;
    end

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
        [v_input,omega_input] = computeVelocitiesFromTracks(omega_l_fine, omega_r_fine, params);
        if strcmp(type_of_ros, 'ros2')
                resp.des_x = states(1,:);
                resp.des_y = states(2,:);
                resp.des_theta = states(3,:);
                resp.des_v = v_input;
                resp.des_omega = omega_input;
                resp.dt = params.dt;
        elseif strcmp(type_of_ros, 'ros1')
            resp.Des_x = states(1,:);
            resp.Des_y = states(2,:);
            resp.Des_theta = states(3,:);
            resp.Des_v = v_input;
            resp.Des_omega = omega_input;
            resp.Dt = params.dt;
        else
            disp('wrong ros version')
        end

        plot_dubins(p0, pf, params);
        fprintf(2,"NEW dubins_optim\n")

        
    elseif strcmp(req.plan_type, "optim")    
        
        solution = optimize_cpp_mex(p0,  pf, omega_l0, omega_r0, t0,  params); 
        plot_solution(solution,p0, pf, params, false);
        %these vectors are too big to transfer to the robot TODO if you fix
        %C++  side then send these otherwise you can have integration
        %errors
         

        if strcmp(type_of_ros, 'ros2')
             resp.des_x = solution.p_fine(1,:);
             resp.des_y = solution.p_fine(2,:);
             resp.des_theta = solution.p_fine(3,:);
             resp.des_v = solution.v_input_fine;
             resp.des_omega = solution.omega_input_fine;
    
            % resp.des_x = solution.p(1,:);
            % resp.des_y = solution.p(2,:);
            % resp.des_theta = solution.p(3,:);
            % resp.des_v = solution.v_input;
            % resp.des_omega = solution.omega_input;
            resp.dt = params.dt;
        elseif strcmp(type_of_ros, 'ros1')
             resp.Des_x = solution.p_fine(1,:);
             resp.Des_y = solution.p_fine(2,:);
             resp.Des_theta = solution.p_fine(3,:);
             resp.Des_v = solution.v_input_fine;
             resp.Des_omega = solution.omega_input_fine;
             resp.Dt = params.dt;
        else
            disp('wrong ros version')
        end

        %solution.Tf
        %length(resp.des_theta)
        disp('Duration Tf')
        solution.Tf
        disp('achieved target')
        solution.achieved_target


    else
        disp("wrong plan type")
    end
    %
    % req.plan_type
    % resp.des_x(end-10:end)
    % resp.des_y(end-10:end)
    % resp.des_theta(end-10:end)
    % resp.des_v(end-10:end)
    % resp.des_omega(end-10:end)
    % resp.dt
end