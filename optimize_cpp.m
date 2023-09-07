
function solution = optimize_cpp(p0,  pf, params) 
    %make it column vector
    p0 = p0(:);
    pf = pf(:);
    %for eval solution
    dt = 0.001; 
    
    % needs to be fixed for code generation
    constr_tolerance = 1e-3;
    
    omega_max = 100; 
    omega_l0 = 40*ones(1,params.N_dyn); %TODO gives issue with 1
    omega_r0 = 40*ones(1,params.N_dyn);
    x0 = [ 10 ,  omega_l0,  omega_r0];
    lb = [ 0  , -omega_max*ones(1,params.N_dyn),  -omega_max*ones(1,params.N_dyn)];
    ub = [ 100, omega_max*ones(1,params.N_dyn),  omega_max*ones(1,params.N_dyn)];


    options = optimoptions('fmincon','Display','iter','Algorithm','sqp',  ... % does not always satisfy bounds
    'MaxFunctionEvaluations', 10000, 'ConstraintTolerance',constr_tolerance);

    tic
    [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost(x, p0,  pf, params), x0,[],[],[],[],lb,ub,  @(x) constraints(x, p0,  pf, params) , options);
    toc

    solution = eval_solution(x, dt,  p0, pf, params) ;
    problem_solved = (EXITFLAG == 1) || (EXITFLAG ==2) ;
    
    %save('test_matlab2.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');



end

