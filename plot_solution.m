function plot_solution(solution,p0, pf, params)


   
    x(1) =solution.Tf;
    x(params.num_params+1:params.num_params+params.N_dyn) =solution.omega_l;
    x(params.num_params+params.N_dyn+1:params.num_params+2*params.N_dyn)=solution.omega_r;
    [c ceq,  solution_constr] = constraints(x, p0(:), pf(:), params);
   

    DEBUG = true;

if (DEBUG)
    
    figure
    subplot(2,1,1)
   
    plot(solution.time,-params.omega_w_max*ones(size(solution.omega_l)),'k'); hold on; grid on;
    plot(solution.time,params.omega_w_max*ones(size(solution.omega_l)),'k');
    plot(solution.time,solution.omega_l,'or-');
    legend({'min','max','opt'});
      ylabel('omega_l')
      
    subplot(2,1,2)

    plot(solution.time,-params.omega_w_max*ones(size(solution.omega_r)),'k'); hold on; grid on;
    plot(solution.time,params.omega_w_max*ones(size(solution.omega_r)),'k');
    plot(solution.time,solution.omega_r,'or-');
    legend({'min','max','opt'});
        ylabel('omega_r')
         
    figure
    subplot(3,1,1)
    plot(solution.time, solution.p(1,:),'r') ; hold on;   grid on; 
    plot(solution_constr.time, solution_constr.p(1,:),'ob') ; hold on;    
    ylabel('X')
    
    subplot(3,1,2)
    plot(solution.time, solution.p(2,:),'r') ; hold on;  grid on;  
    plot(solution_constr.time, solution_constr.p(2,:),'ob') ; hold on;    
    ylabel('Y')
    
    subplot(3,1,3)
    plot(solution.time, solution.p(3,:),'r') ; hold on; grid on;   
    plot(solution_constr.time, solution_constr.p(3,:),'ob') ; hold on;
    ylabel('theta')
       
    
end
    figure
    plot_curve( solution,solution_constr, p0(:), pf(:));

    
end