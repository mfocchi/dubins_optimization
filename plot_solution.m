function plot_solution(solution,p0, pf, params)


   
    x(1) =solution.Tf;
    x(params.num_params+1:params.num_params+params.N_dyn) =solution.omega_l;
    x(params.num_params+params.N_dyn+1:params.num_params+2*params.N_dyn)=solution.omega_r;
    [c ceq,  solution_constr] = constraints(x, p0(:), pf(:), params);
   

    DEBUG = true;

if (DEBUG)
    
    figure
   
    plot(solution.time,-params.omega_w_max*ones(size(solution.omega_l)),'k'); hold on; grid on;
    plot(solution.time,params.omega_w_max*ones(size(solution.omega_l)),'k');
    plot(solution.time,solution.omega_l,'or-');
    plot(solution.time,solution.omega_r,'ob-');
    legend({'min','max','omega_w_l','omega_w_r'});
    ylabel('omega_w')
  
         
    figure
    subplot(3,1,1)
    plot(solution.time, solution.p(1,:),'r') ; hold on;   grid on; 
    plot(solution.solution_constr.time, solution.solution_constr.p(1,:),'ob') ; hold on;    
    ylabel('X')
    
    subplot(3,1,2)
    plot(solution.time, solution.p(2,:),'r') ; hold on;  grid on;  
    plot(solution.solution_constr.time, solution.solution_constr.p(2,:),'ob') ; hold on;    
    ylabel('Y')
    
    subplot(3,1,3)
    plot(solution.time, solution.p(3,:),'r') ; hold on; grid on;   
    plot(solution.solution_constr.time, solution.solution_constr.p(3,:),'ob') ; hold on;
    ylabel('theta')
  
          
    figure
    subplot(2,1,1)
    plot(solution.solution_constr.time,params.v_max*ones(size(solution.solution_constr.v_input)),'k-'); hold on; grid on;
    plot(solution.solution_constr.time,params.v_min*ones(size(solution.solution_constr.v_input)),'k-');
    plot(solution.solution_constr.time, solution.solution_constr.v_input,'ob-') ; hold on;    
    ylabel('v input')
    
    subplot(2,1,2)
    plot(solution.solution_constr.time,params.omega_max*ones(size(solution.solution_constr.omega_input)),'k-'); hold on; grid on;
    plot(solution.solution_constr.time,params.omega_min*ones(size(solution.solution_constr.omega_input)),'k-');
    plot(solution.solution_constr.time, solution.solution_constr.omega_input,'ob-') ; hold on;    
    ylabel('omega input')
    
  
    
    
end
    figure
    plot_curve( solution,solution_constr, p0(:), pf(:));

    
end