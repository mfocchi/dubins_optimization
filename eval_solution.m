function solution = eval_solution(x,  dt, p0, pf, params)

    Tf =  x(1);
    omega_l = x(params.num_params+1:params.num_params+params.N_dyn); 
    omega_r = x(params.num_params+params.N_dyn+1:params.num_params+2*params.N_dyn); 

    % check they are column vectors
    p0 = p0(:);
    pf = pf(:);
    % variable intergration step
    dt_dyn = Tf / (params.N_dyn-1); 

    % single shooting
    [states, t] = computeRollout(p0, 0,dt_dyn, params.N_dyn, omega_l, omega_r, params);
    x = states(1,:);
    y = states(2,:);
    theta = states(3,:);

    p_0 = [x(:,1); y(:,1); theta(:,1)];
    p_f = [x(:,end); y(:,end); theta(:,end)];

    % make sure they are column vectors
    p0 = p0(:);
    pf = pf(:);

    % init struct foc C++ code generation
    solution = struct;

    %compute path
    % deltax = diff(p(1,:));  % diff(X);
    % deltay = diff(p(2,:));   % diff(Y);
    % deltaz = diff(p(3,:));    % diff(Z);
    % solution.path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));

    solution.initial_error = norm(p_0 -p0);
    solution.final_error_real = norm(p_f -pf);
    solution.omega_r  = omega_r;
    solution.omega_l  = omega_l;
    solution.p =  [x; y; theta];
    solution.time = t;
    solution.Tf = Tf;
    solution.achieved_target =  [x(end); y(end); theta(end)];

    
end