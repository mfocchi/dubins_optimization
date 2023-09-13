
function solution = optimize_cpp(p0,  pf, omega_l0, omega_r0, t0, params) 
    %make it column vector
    p0 = p0(:);
    pf = pf(:);
    %for eval solution
    dt = 0.001; 
    
    % needs to be fixed for code generation
    constr_tolerance = 1e-3;

    [v_input,omega_input] =  computeVelocitiesFromTracks(omega_l0, omega_r0, params);
    if  any(v_input >params.v_max) 
        disp('initialization is unfeasible')
    end
    
    x0 = [ t0 ,  omega_l0,  omega_r0];
    lb = [ 0  , -params.omega_w_max*ones(1,params.N_dyn),  -params.omega_w_max*ones(1,params.N_dyn)];
    ub = [ params.t_max, params.omega_w_max*ones(1,params.N_dyn),  params.omega_w_max*ones(1,params.N_dyn)];
    options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
    'MaxFunctionEvaluations', 100000, 'ConstraintTolerance',constr_tolerance);

    tic
    [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf, params), x0,[],[],[],[],lb,ub,  @(x) constraints(x, p0,  pf, params) , options);
    toc

    

    solution = eval_solution(x, dt,  p0, pf, params) ;
    solution.x = x;
    solution.cost = final_cost;
    solution.problem_solved = EXITFLAG ;%(EXITFLAG == 1) || (EXITFLAG == 2);
    % 1 First-order optimality measure was less than options.OptimalityTolerance, and maximum constraint violation was less than options.ConstraintTolerance.
    % 0 Number of iterations exceeded options.MaxIterations or number of function evaluations exceeded options.MaxFunctionEvaluations.
    % -1 Stopped by an output function or plot function.
    % -2 No feasible point was found.
    % 2 Change in x was less than options.StepTolerance (Termination tolerance on x, a scalar, the default is 1e-10) and maximum constraint violation was less than options.ConstraintTolerance.


    solution.optim_output = output;
    % evaluate constraint violation 
    [c ceq, solution_constr] = constraints(x, p0,  pf, params);
    solution.c = c;    
    solution.solution_constr = solution_constr;
   
 
end

