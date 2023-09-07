clear all; close all

%cd to actual dir
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);


% INITIAL STATE (X,Y, THETA)
p0 = [0.0, 0.0, -0.3]; 
%FINAL STATE  (X,Y, THETA)
pf = [10.0, 10.0, -1.8]; 

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


% solution1 = optimize_cpp(p0,  pf, params) 
% solution1.Tf
% solution1.achieved_target
% plot_solution(solution1, p0, pf, params) 

% generates the cpp code
% run the mex generator after calling optimize_cpp otherwise he complains it is missing the pa1 
% cfg = coder.config('mex');
% cfg.IntegrityChecks = false;
% cfg.SaturateOnIntegerOverflow = false;
% codegen -config cfg  optimize_cpp -args {[0, 0, 0], [0, 0, 0], coder.cstructname(params, 'param') } -nargout 1 -report

%it gives a slightly different result than optimal_control_2ropes:
%solution.Tf = 1.3234
%solution.achieved_target(normal test) = 0.5971     3.9923  -4.0035
solution = optimize_cpp_mex(p0,  pf, params);
solution.problem_solved
solution.Tf
solution.achieved_target
plot_solution(solution, p0, pf, params) 


%save('../simulation/compact_model/tests/test_matlab2_cpp.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');
%system('python3 test_mex.py');
