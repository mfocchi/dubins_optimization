clear all ; close all ; clc

%cd to actual dir if you are not already there
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);

SAVE_TRAJ = false;
USEGENCODE = true;
DEBUG = true; %shows local orientation of trajectory and all the plots

% INITIAL STATE (X,Y, THETA)
p0 = [0.0; 0.0; -0.]; 
%FINAL STATE  (X,Y, THETA)
pf = [-0.4758; -1.1238; 0.9638];

run('robot_params.m');

% % do init with dubins
dubConnObj = dubinsConnection;
curvature_max = params.omega_max/params.v_max;
dubConnObj.MinTurningRadius = 1/curvature_max;
[pathSegObj, pathCosts] = connect(dubConnObj,p0',pf');
%show(pathSegObj{1})
% get total time
t0 = sum(pathSegObj{1}.MotionLengths)/params.v_max;
% compute the omega from dubin
omegas = get_omega_from_dubins(pathSegObj{1}, params.v_max, 1/curvature_max);
%map to wheel omega
[omega_l0, omega_r0, t_rough] = getVelocityParamsFromDubin(params, pathSegObj{1}.MotionLengths, omegas);

% normal init  (no longer used)
% t0 = norm(pf(1:2) - p0(1:2))/(0.5*params.v_max); 
% omega_l0 = 0.*params.omega_w_max*ones(1,params.N_dyn); %TODO gives issue with 0, should be initialized with half
% omega_r0 = 0.*params.omega_w_max*ones(1,params.N_dyn);
% 

%gen code (run if you did some change in the cost)
if ~isfile('optimize_cpp_mex.mexa64')
    disp('Generating C++ code');
    cfg = coder.config('mex');
    cfg.IntegrityChecks = false;
    cfg.SaturateOnIntegerOverflow = false;
    codegen -config cfg  optimize_cpp -args {zeros(3,1), zeros(3,1),coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]),0, coder.cstructname(params, 'param') } -nargout 1 -report
end

mpc_fun   = 'optimize_cpp';
if USEGENCODE
    mpc_fun=append(mpc_fun,'_mex' );
end
mpc_fun_handler = str2func(mpc_fun);

solution = mpc_fun_handler(p0,  pf, omega_l0, omega_r0, t0,  params); 
solution.problem_solved
solution.Tf
solution.achieved_target

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

[final_cost, cost_components] = cost(solution.x, p0,  pf, params);
fprintf('target constraint violated:  %f\n\n',solution.c(1));
constr_viol = solution.c(2:end)>constr_tolerance;
num_constr_viol = sum(constr_viol);
if num_constr_viol>0
    index_violated =find(constr_viol);
    outputstr = repmat('%i ', 1, num_constr_viol); % replicate it to match the number of columns
    fprintf(strcat('path constraints violated at index:  ', outputstr,'\n'),index_violated);
else
    fprintf('path constraints violated:  %i\n\n',num_constr_viol);
end
fprintf('cost:  %f\n\n',final_cost);
fprintf('cost component: time: %f,  smoothing : %f  \n \n',cost_components.time,  cost_components.smoothing_speed);
fprintf('target error_real:  %f\n\n',solution.final_error_real)
fprintf('target error discrete:  %f\n\n', solution.solution_constr.final_error_discrete)
fprintf('max_integration_error:  %f\n\n', solution.final_error_real - solution.solution_constr.final_error_discrete)
fprintf('duration:  %f\n\n', solution.Tf)


% for debug
%s = eval_solution(solution.x, dt,  p0, pf, params) ;
%plot_solution(s,p0, pf, params, true);

plot_solution(solution,p0, pf, params, DEBUG);

%% Save data for testing on robot
% generate a velocity profile by interpolation. User must select a desired
% step size to define the resolution. In case of integration with ROS, it
% should correspond to the control execution period.

% if(SAVE_TRAJ)
%     dt = 0.002; % [s]
%     filename = 'optimal_traj_2_fine_13_10.csv';
%     opts.Interpreter = 'none';
%     opts.Default = 'Cancel';
%     answer = questdlg(strcat('Do you want to save a velocity trajectory as "', filename, '" ?'), ...
% 	    'Save', ...
% 	    'Yes','Cancel', opts);
%     % Handle response
%     switch answer
%         case 'Yes'
%             disp("Save table as '" + filename + "'");
%             doretta_path = extractUnicycleSetpoint(solution, dt);
%             writetable(doretta_path, filename);
%         case 'Cancel'
%             disp("Table was not saved");
%     end
% end
