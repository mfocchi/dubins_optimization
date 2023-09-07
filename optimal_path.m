clear all ; close all ; clc

%cd to actual dir if you are not already there
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);


% INITIAL STATE (X,Y, THETA)
p0 = [0.0; 0.0; -0.]; 
%FINAL STATE  (X,Y, THETA)
pf = [10.0; 10.0; -1.8]; 

params.int_method = 'euler';
params.N_dyn = 30; %dynamic constraints (number of knowts in the discretization) 
params.int_steps = 5 ;%cast(5,"int64"); %0 means normal intergation
params.num_params = 1; %final time

params.w1= 0;  % minimum time 
params.w2= 1; % smoothing  
params.w3= 0; %soft tracking of end target (is already in the constraints not needed)
params.w4= 0; % invariant set TODO


params.omega_w_max = 100;
params.omega_max = 100;
params.v_max = 100;
params.v_min = 0;
params.width = 0.606; % [m]
params.sprocket_radius = 0.0979; % [m]
params.gearbox_ratio = 39.4;
params.slip_fit_coeff.left  = [-0.0591,   -0.2988];
params.slip_fit_coeff.right = [0.0390,    0.2499 ];

constr_tolerance = 1e-3;

dt=0.001; % only to evaluate solution
 
omega_l0 = 0.5*params.omega_w_max*ones(1,params.N_dyn); %TODO gives issue with 0, should be initialized with half
omega_r0 = 0.5*params.omega_w_max*ones(1,params.N_dyn);
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
solution.cost = final_cost;

problem_solved = (EXITFLAG == 1) || (EXITFLAG == 2);

if problem_solved
    fprintf(2,"Problem converged!\n")
else 
    fprintf(2,"Problem didnt converge!\n")
end

 
fprintf('cost:  %f\n\n',solution.cost)
fprintf('final_error_real:  %f\n\n',solution.final_error_real)
fprintf('final_error_discrete:  %f\n\n', solution_constr.final_error_discrete)
fprintf('max_integration_error:  %f\n\n', solution.final_error_real - solution_constr.final_error_discrete)
fprintf('duration:  %f\n\n', solution.Tf)


plot_solution(solution,p0, pf, params);

save('traj.mat','solution', 'p0','pf');
