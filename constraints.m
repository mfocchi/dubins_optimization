function [ineq, eq, solution_constr] = constraints(x,   p0,  pf, params)

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

    % size not known
    ineq = zeros(1,0);

    solution_constr.p = [x;y;theta];
    solution_constr.time = t;
    solution_constr.final_error_discrete = norm(p_f - pf);

    % ineq are <= 0
    fixed_slack = 0.02;%*norm(p0 - pf); 
    
    % pass from target at Tf
    ineq= [ineq norm(p_f - pf) - fixed_slack];
  
    %limits on v  omega
    [v_input,omega_input] =  computeVelocitiesFromTracks(omega_l, omega_r, params);
%     

%     
%     for i=1:params.N_dyn     
%          ineq = [ineq  -omega_input(i)  -params.omega_max ];  % omega  > omega_min ->  omega_min - omega <0
% 
%     end 
% 
%     for i=1:params.N_dyn     
%          ineq = [ineq  -v_input(i) params.v_min ]; % v  > v_min ->  v_min - v <0
%     end 
%     for i=1:params.N_dyn     
%          ineq = [ineq  v_input(i) -params.v_max ]; % v  < v_max   ->    v - v_max<0
%     end 
% %     
%     for i=1:params.N_dyn     
%          ineq = [ineq  omega_input(i)  -params.omega_max ];% omega  < omega_max  ->    omega - omega_max<0
%     end 
    
%     
   eq = [];
%     for i=1:params.N_dyn     
%          eq = [eq  v_input(i)  -params.omega_v_max ];% omega  < omega_max  ->    omega - omega_max<0
%     end 

 
 %


    % if any(isinf(ineq))
    %     disp('Infn in constraint')
    %     find(isinf(ineq)) 
    %     isinf(ineq)
    % end
    % if any(isnan(ineq))
    %     disp('Nan in constraint')
    %     find(isnan(ineq))
    %     isnan(ineq)
    % end

end