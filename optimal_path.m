clear all ; close all ; clc

%cd to actual dir if you are not already there
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);


USEGENCODE = false;
GENCODE = false;

if GENCODE
    %generates the cpp code
    %run the mex generator after calling optimize_cpp otherwise he complains it is missing the pa1 
    cfg = coder.config('mex');
    cfg.IntegrityChecks = false;
    cfg.SaturateOnIntegerOverflow = false;
    codegen -config cfg  optimize_cpp -args {zeros(3,1), zeros(3,1),coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]), coder.cstructname(params, 'param') } -nargout 1 -report
end


% INITIAL STATE (X,Y, THETA)
p0 = [0.0; 0.0; -0.3]; 
%FINAL STATE  (X,Y, THETA)
pf = [10.0; 10.0; -1.]; 

params.int_method = 1; %1 ='euler', 2 =  'rk4';
params.N_dyn = 20; %dynamic constraints (number of knowts in the discretization) 
params.int_steps = 10 ;%cast(5,"int64"); %0 means normal intergation
params.num_params = 1; %final time

params.w1= 1;  % minimum time 
params.w2= 1; % smoothing  
params.w3= 0; %soft tracking of end target (is already in the constraints not needed)
params.w4= 0; % invariant set TODO


params.omega_w_max = 1000;
params.omega_max = 1;
params.omega_min = -1;
params.v_max = 1.5;
params.v_min = -1.5;
params.VELOCITY_LIMITS = false;

params.width = 0.606; % [m]
params.sprocket_radius = 0.0979; % [m]
params.gearbox_ratio = 39.4;
params.slip_fit_coeff.left  = [-0.0591,   -0.2988];
params.slip_fit_coeff.right = [0.0390,    0.2499 ];
omega_l0 = 0.1*params.omega_w_max*ones(1,params.N_dyn); %TODO gives issue with 0, should be initialized with half
omega_r0 = 0.1*params.omega_w_max*ones(1,params.N_dyn);


if ~USEGENCODE     
    constr_tolerance = 1e-5;
    dt=0.001; % only to evaluate solution

    [v_input,omega_input] =  computeVelocitiesFromTracks(omega_l0, omega_r0, params);
    if  any(v_input >params.v_max) 
        disp('initialization is unfeasible')
    end
    x0 = [ 20 ,  omega_l0,  omega_r0];
    lb = [ 0  , -params.omega_w_max*ones(1,params.N_dyn),  -params.omega_w_max*ones(1,params.N_dyn)];
    ub = [ 100, params.omega_w_max*ones(1,params.N_dyn),  params.omega_w_max*ones(1,params.N_dyn)];
    options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
    'MaxFunctionEvaluations', 10000, 'ConstraintTolerance',constr_tolerance);
    tic
    [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf, params), x0,[],[],[],[],lb,ub,  @(x) constraints(x, p0,  pf, params) , options);
    toc

    % evaluate constraint violation 
    [c ceq, solution_constr] = constraints(x, p0,  pf, params);

    solution = eval_solution(x, dt,  p0, pf, params) ;
    solution.x = x;
    solution.cost = final_cost;
    solution.problem_solved = (EXITFLAG == 1) || (EXITFLAG == 2);
    solution.solution_constr = solution_constr;    
       
else 
    % solution1 = optimize_cpp(p0,  pf, params) 
    % solution1.Tf
    % solution1.achieved_target

    %it gives a slightly different result than optimize_cpp:
    solution = optimize_cpp_mex(p0,  pf,omega_l0, omega_r0, params); %I take transpose because it wants row inputs
    solution.problem_solved
    solution.Tf
    solution.achieved_target

end    

if solution.problem_solved
    fprintf(2,"Problem converged!\n")
else 
    fprintf(2,"Problem didnt converge!\n")
end

fprintf('cost:  %f\n\n',solution.cost)
fprintf('final_error_real:  %f\n\n',solution.final_error_real)
fprintf('final_error_discrete:  %f\n\n', solution.solution_constr.final_error_discrete)
fprintf('max_integration_error:  %f\n\n', solution.final_error_real - solution.solution_constr.final_error_discrete)
fprintf('duration:  %f\n\n', solution.Tf)

plot_solution(solution,p0, pf, params);

save('traj.mat','solution', 'p0','pf');
