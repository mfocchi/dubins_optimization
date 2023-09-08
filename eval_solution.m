function solution = eval_solution(x,  dt, p0, pf, params)

    Tf =  x(1);
    omega_l = x(params.num_params+1:params.num_params+params.N_dyn); 
    omega_r = x(params.num_params+params.N_dyn+1:params.num_params+2*params.N_dyn); 

    % check they are column vectors
    p0 = p0(:);
    pf = pf(:);

     
  
    % resample inputs 
    n_samples = floor(Tf/dt);
    omega_l_fine = zeros(1,n_samples);
    omega_r_fine = zeros(1,n_samples);
    rough_count = 1;
    t_ = 0;
    for i=1: n_samples
       t_= t_+dt;
       if t_>= ((n_samples) *dt/(params.N_dyn-1))
            rough_count = rough_count + 1;
            t_ =0;
        end  
        omega_l_fine(i) =  omega_l(rough_count);
        omega_r_fine(i) =  omega_r(rough_count);
    end

   % single shooting
   
    % course integration
    %dt_dyn = Tf / (params.N_dyn-1); 
    % fine integration 
    %[states_rough, t_rough] = computeRollout(p0, 0,dt_dyn, params.N_dyn, omega_l, omega_r, params);
    params.int_steps = 0;
    [states, t] = computeRollout(p0, 0,dt, n_samples, omega_l_fine, omega_r_fine, params);
    
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

    
    solution.omega_l_fine  = omega_l_fine;
    solution.omega_r_fine  = omega_r_fine;    
    solution.final_error_real = norm(p_f -pf);
    solution.p =  [x; y; theta];
    solution.time = t;
    solution.Tf = Tf;
    solution.achieved_target =  [x(end); y(end); theta(end)];

    
end