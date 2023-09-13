clear all ; close all ; clc

%cd to actual dir if you are not already there
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);


USEGENCODE = true;
GENCODE = false;


% INITIAL STATE (X,Y, THETA)
p0 = [0.0; 0.0; -0.3]; 
%FINAL STATE  (X,Y, THETA)
pf = [1; 1.0; -1.9];%pf = [10.0; 10.0; -1.9];
%pf = [1; -2.0; -1.9];%pf = [10.0; 10.0; -1.9];


params.int_method = 2; %1 ='euler', 2 =  'rk4';
params.N_dyn = 40; %dynamic constraints (number of knowts in the discretization) 
params.int_steps = 15 ;%cast(5,"int64"); %0 means normal intergation
params.num_params = 1; %final time

params.w1= 1;  % minimum time 
params.w2= 1000; % smoothing   1000
params.w3= 0; %soft tracking of end target (is already in the constraints not needed)
params.w4= 0; % invariant set TODO

params.UNICYCLE = true;
params.omega_w_max = 2000;
params.omega_max = 5.5;
params.omega_min = -5.5;
params.v_max = 1;
params.v_min = 1;%-2.5;
params.VELOCITY_LIMITS = true;
params.t_max = 80;
params.slack_target = 0.02;
constr_tolerance = 1e-3;

params.width = 0.606; % [m]
params.sprocket_radius = 0.0979; % [m]
params.gearbox_ratio = 39.4;
params.slip_fit_coeff.left  = [-0.0591,   -0.2988];
params.slip_fit_coeff.right = [0.0390,    0.2499 ];
omega_l0 = 0.0*params.omega_w_max*ones(1,params.N_dyn); %TODO gives issue with 0, should be initialized with half
omega_r0 = 0.0*params.omega_w_max*ones(1,params.N_dyn);
t0 = norm(pf(1:2) - p0(1:2))/(0.5*params.v_max);  

if GENCODE
    %generates the cpp code
    %run the mex generator after calling optimize_cpp otherwise he complains it is missing the pa1 
    cfg = coder.config('mex');
    cfg.IntegrityChecks = false;
    cfg.SaturateOnIntegerOverflow = false;
    codegen -config cfg  optimize_cpp -args {zeros(3,1), zeros(3,1),coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]),0, coder.cstructname(params, 'param') } -nargout 1 -report
end

if ~USEGENCODE     

    dt=0.001; % only to evaluate solution

    [v_input,omega_input] =  computeVelocitiesFromTracks(omega_l0, omega_r0, params);
    if  any(v_input >params.v_max) 
        disp('initialization is unfeasible')
    end      
    x0 = [ t0 ,  omega_l0,  omega_r0];
    lb = [ 0  , -params.omega_w_max*ones(1,params.N_dyn),  -params.omega_w_max*ones(1,params.N_dyn)];
    ub = [ params.t_max, params.omega_w_max*ones(1,params.N_dyn),  params.omega_w_max*ones(1,params.N_dyn)];
    options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
    'MaxFunctionEvaluations', 10000, 'ConstraintTolerance',constr_tolerance);
    tic
    [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf, params), x0,[],[],[],[],lb,ub,  @(x) constraints(x, p0,  pf, params) , options);
    toc  

    solution = eval_solution(x, dt,  p0, pf, params) ;
    solution.x = x;
    solution.cost = final_cost;
    solution.problem_solved = (EXITFLAG == 1) || (EXITFLAG == 2);
    
    % evaluate constraint violation 
    [c ceq, solution_constr] = constraints(solution.x, p0,  pf, params);
    solution.c =c;
    solution.solution_constr = solution_constr;    
       
else 
    %solution = optimize_cpp(p0,  pf,omega_l0, omega_r0, t0, params); 
    %it gives a slightly different result than optimize_cpp:
    solution = optimize_cpp_mex(p0,  pf,omega_l0, omega_r0, t0,  params); 
    solution.problem_solved
    solution.Tf
    solution.achieved_target

end    
% 
% 1 First-order optimality measure was less than options.OptimalityTolerance, and maximum constraint violation was less than options.ConstraintTolerance.
% 0 Number of iterations exceeded options.MaxIterations or number of function evaluations exceeded options.MaxFunctionEvaluations.
% -1 Stopped by an output function or plot function.
% -2 No feasible point was found.
% 2 Change in x was less than options.StepTolerance and maximum constraint violation was less than options.ConstraintTolerance.

switch solution.problem_solved
    case 1 
        fprintf(2,"Problem converged!\n")
    case -2  
        fprintf(2,"Problem didnt converge!\n")
    case 2 
        fprintf(2,"semidefinite solution (should modify the cost)\n")
    case 0 
        fprintf(2,"Max number of feval exceeded (10000)\n")
end


fprintf(2,"number of iterations: %i\n", solution.optim_output.iterations);
fprintf(2,"number of func evaluations: %i\n", solution.optim_output.funcCount);

[cost, cost_components] = cost(solution.x, p0,  pf, params);
fprintf('target constraint violated:  %f\n\n',solution.c(1));
constr_viol = solution.c(2:end)>constr_tolerance;
num_constr_viol = sum(constr_viol);
if num_constr_viol>0
    index_violated =find(constr_viol);
    outputstr = repmat('%i ', 1, num_constr_viol) % replicate it to match the number of columns
    fprintf(strcat('path constraints violated at index:  ', outputstr,'\n'),index_violated);
else
    fprintf('path constraints violated:  %i\n\n',num_constr_viol);
end
fprintf('cost:  %f\n\n',solution.cost);
fprintf('cost component: time: %f,  smoothing : %f  \n \n',cost_components.time,  cost_components.smoothing);
fprintf('target error_real:  %f\n\n',solution.final_error_real)
fprintf('target error discrete:  %f\n\n', solution.solution_constr.final_error_discrete)
fprintf('max_integration_error:  %f\n\n', solution.final_error_real - solution.solution_constr.final_error_discrete)
fprintf('duration:  %f\n\n', solution.Tf)
plot_solution(solution,p0, pf, params);

save('traj.mat','solution', 'p0','pf');
