clear all ; close all ; clc

%cd to actual dir if you are not already there
filePath = matlab.desktop.editor.getActiveFilename;
pathparts = strsplit(filePath,filesep);
dirpath= pathparts(1:end-1);
actual_dir =  strjoin(dirpath,"/");
cd(actual_dir);


USEGENCODE = true;
GENCODE = false;
DEBUG = false;

% INITIAL STATE (X,Y, THETA)
p0 = [0.0; 0.0; -0.3]; 
%FINAL STATE  (X,Y, THETA)
%pf = [1; 1.0; -1.9];%pf = [10.0; 10.0; -1.9]; % works with UNICYCLE TODO
%%does not converge with LONGSLIP
%pf = [1; -2.0; -1.9];%pf = [10.0; 10.0; -1.9];
pf = [1.5; 1.0; -1.2];%pf = [10.0; 10.0; -1.9]; % works with LONGSLIP


params.int_method = 'rk4'; %'eul'  'rk4';
params.N_dyn = 40; %dynamic constraints (number of knowts in the discretization) 
params.int_steps = 15 ;%cast(5,"int64"); %0 means normal intergation
params.num_params = 1; %final time

params.w1= 0;  % minimum time 
params.w2= 1; % smoothing   1000
params.w3= 0; %soft tracking of end target (is already in the constraints not needed)
params.w4= 0; % invariant set TODO


params.model = 'UNICYCLE';
%params.model =  'LONGSLIP';
%params.model = 'SIDESLIP';
params.omega_w_max = 2000;
params.omega_max = 5.5;
params.omega_min = -5.5;
params.v_max = 1;
params.v_min = 1;%-2.5;
params.VELOCITY_LIMITS = true;
params.t_max = 80;
params.slack_target = 0.02;
constr_tolerance = 1e-3;
dt=0.001; % only to evaluate solution

params.width = 0.606; % [m]
params.sprocket_radius = 0.0979; % [m]
params.gearbox_ratio = 39.4;
params.slip_fit_coeff.left  = [-0.0591,   -0.2988];
params.slip_fit_coeff.right = [0.0390,    0.2499 ];
params.side_slip_fit_coeff = [-0.7758   -6.5765];
params.slip_fit_coeff.min_value = -1000;


t0 = norm(pf(1:2) - p0(1:2))/(0.5*params.v_max); 
omega_l0 = 0.5*params.omega_w_max*ones(1,params.N_dyn); %TODO gives issue with 0, should be initialized with half
omega_r0 = 0.5*params.omega_w_max*ones(1,params.N_dyn);
 

% % do init with dubins
% dubinsObj= dubinsClass;
% % Find optimal Dubins solution
% [pidx, curve, k, lengths] = dubinsObj.dubins_shortest_path(p0(1), p0(2), p0(3), pf(1), pf(2), pf(3), params.omega_max);
% 
% % build up velocity trajectories
% sample_instants = floor(lengths/sum(lengths)*params.N_dyn);
% omega0 = 0.*ones(1,params.N_dyn);
% v0 = params.v_max.*ones(1,params.N_dyn);
% idx1 = sample_instants(1);
% idx2 = sample_instants(1)+sample_instants(2);
% idx3 = sum(sample_instants);
% omega0(1:idx1) = k(1);
% omega0(idx1+1:idx2) = k(2);
% omega0(idx2+1:idx3) = k(3);
% r = params.sprocket_radius;
% B = params.width;
% A = [r/2, r/2; r/B, -r/B];

% for i=1:params.N_dyn
%   y = inv(A)*[v0(i); omega0(i)];
%   omega_r0(i) = params.gearbox_ratio*y(1); 
%   omega_l0(i) = params.gearbox_ratio* y(2);
% end

%     dt = 0.001;
%     % resample inputs 
%     n_samples = floor(t0/dt);
%     omega_l_fine = zeros(1,n_samples);
%     omega_r_fine = zeros(1,n_samples);
%     rough_count = 1;
%     t_ = 0;
%     for i=1: n_samples
%        t_= t_+dt;
%        if t_>= ((n_samples) *dt/(params.N_dyn-1))
%             rough_count = rough_count + 1;
%             t_ =0;
%         end  
%         omega_l_fine(i) =  omega_l0(rough_count);
%         omega_r_fine(i) =  omega_r0(rough_count);
%     end
% 
%     params.int_steps = 0;
%     [states, t] = computeRollout(p0, 0,dt, n_samples, omega_l_fine, omega_r_fine, params);
%     
%     x = states(1,:);
%     y = states(2,:);
%     theta = states(3,:);
% 
% p =  [x; y; theta];
%   figure
% plot(pf(1), pf(2) , 'Marker', '.', 'Color','r', 'MarkerSize',60) ;
% min_x = min(min(p(1,:)), pf(1))-3 ;
% max_x = max(max(p(1,:)), pf(1))+3 ;
% min_y = min(min(p(2,:)), pf(2))-3 ;
% max_y = max(max(p(2,:)), pf(2)) +3 ;
% 
% % set(gca,'XLim',[min_x max_x])
% % set(gca,'YLim',[min_y max_y])
% 
% color_input = 'r'; 
% 
% % actual traj
% plot(p(1,:), p(2,:), 'Color', color_input ) ;
% 
% 
% scaling = 0.1*norm(pf(1:2) - p0(1:2));
% %initial orient
% plotOrientation([p0(1); p0(2)], p0(3), scaling);
% %final orient 
% plotOrientation([pf(1); pf(2)], pf(3), 0.1);
% grid on;
% 
% xlabel('X');
% ylabel('Y');

%with matlab
% dubConnObj = dubinsConnection;
% dubConnObj.MinTurningRadius = 1/params.omega_max;
% [pathSegObj, pathCosts] = connect(dubConnObj,p0',pf');
% show(pathSegObj{1})



if GENCODE
    %generates the cpp code
    %run the mex generator after calling optimize_cpp otherwise he complains it is missing the pa1 
    cfg = coder.config('mex');
    cfg.IntegrityChecks = false;
    cfg.SaturateOnIntegerOverflow = false;
    codegen -config cfg  optimize_cpp -args {zeros(3,1), zeros(3,1),coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]),0, coder.cstructname(params, 'param') } -nargout 1 -report
end

if ~USEGENCODE     

  

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
    outputstr = repmat('%i ', 1, num_constr_viol); % replicate it to match the number of columns
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


% for debug
%s = eval_solution(solution.x, dt,  p0, pf, params) ;
%plot_solution(s,p0, pf, params, true);

plot_solution(solution,p0, pf, params, DEBUG);

%save('traj.mat','solution', 'p0','pf');
