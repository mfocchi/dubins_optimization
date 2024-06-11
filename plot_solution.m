function plot_solution(solution,p0, pf, params, DEBUG)

 if nargin < 5
    DEBUG=false;
 end


if (DEBUG)
    

    figure   
    plot(solution.time,-params.omega_w_max*ones(size(solution.omega_l)),'k'); hold on; grid on;
    plot(solution.time,params.omega_w_max*ones(size(solution.omega_l)),'k');
    plot(solution.time,solution.omega_l,'or-');
    plot(solution.time,solution.omega_r,'ob-');
    legend({'min','max','omega_w_l','omega_w_r'});
    ylabel('omega_w','interpreter','none')
         
    figure
    %plots of the states
    subplot(3,1,1)
    plot(solution.time_fine, solution.p_fine(1,:),'r') ; hold on;   grid on; 
    plot(solution.time, solution.p(1,:),'ob') ; hold on;    
    ylabel('X')
    
    subplot(3,1,2)
    plot(solution.time_fine, solution.p_fine(2,:),'r') ; hold on;  grid on;  
    plot(solution.time, solution.p(2,:),'ob') ; hold on;    
    ylabel('Y')
    
    subplot(3,1,3)
    plot(solution.time_fine, solution.p_fine(3,:),'r') ; hold on; grid on;   
    plot(solution.time, solution.p(3,:),'ob') ; hold on;
    ylabel('theta')
  
          
    figure
    %plots of v omega
    subplot(2,1,1)
    plot(solution.time,params.v_max*ones(size(solution.v_input)),'k-'); hold on; grid on;
    plot(solution.time,params.v_min*ones(size(solution.v_input)),'k-');
    plot(solution.time, solution.v_input,'ob-') ; hold on;    
    ylabel('v input')
    
    subplot(2,1,2)
    plot(solution.time,params.omega_max*ones(size(solution.omega_input)),'k-'); hold on; grid on;
    plot(solution.time,params.omega_min*ones(size(solution.omega_input)),'k-');
    plot(solution.time, solution.omega_input,'ob-') ; hold on;    
    ylabel('omega input')
    

    
    if ~strcmp(params.model,'UNICYCLE')
        
        [R, beta_l, beta_r, alpha] = evalSlippage(solution.omega_l, solution.omega_r, params);          
        
        figure
        subplot(3,1,1)
        plot(solution.time, alpha,'ro-') ; hold on;    grid on;
        ylabel('alpha')
        
        subplot(3,1,2)
        plot(solution.time, beta_l,'ro-') ; hold on;    grid on;
        plot(solution.time, beta_r,'bo-') ;    
        legend({'beta_l','beta_r'});
        ylabel('slippage')
        
        subplot(3,1,3)
        plot(solution.time, R,'ro-') ; hold on;    grid on;
        ylim([-1 1]);
        ylabel('radius')
      
       
        
    
    end
    
end

figure;
%DUBINS
%1- plots the optimization curve
plot_curve( solution, p0(:), pf(:), DEBUG);
shg

%luigi
% curvature_max = params.omega_max/params.v_max;
% dubinsObj= dubinsClass;
% %Find optimal Dubins solution
% [pidx, curve] = dubinsObj.dubins_shortest_path(p0(1), p0(2), p0(3), pf(1), pf(2), pf(3), curvature_max);
% %plot dubin
% dubinsObj.plotdubins(curve, true, [1 0 0], [0 0 0], [1 0 0]); hold on;
% 
% 
%matlab plot after integration
dubConnObj = dubinsConnection;
curvature_max = params.omega_max/params.v_max; %using max forward speed
dubConnObj.MinTurningRadius = 1/curvature_max;
[pathSegObj, pathCosts] = connect(dubConnObj,p0',pf');
omegas = get_omega_from_dubins(pathSegObj{1}, params.v_max,dubConnObj.MinTurningRadius);
[omega_l0, omega_r0, time] = getWheelVelocityParamsFromDubin(params, pathSegObj{1}.MotionLengths, omegas, params.dt);
numbe_of_samples = length(omega_l0);
params.int_steps = 0;
params.model = 'UNICYCLE'; %need to set unicycle!!!
%integrate the fine grid omegas
[states, t] = computeRollout(p0, 0,params.dt, numbe_of_samples, omega_l0, omega_r0, params);
x = states(1,:);
y = states(2,:);
theta = states(3,:);
p =  [x; y; theta];
color_input = 'k'; 
% actual dubins traj obtained by fine integration (black line)
plot(p(1,:), p(2,:), '-.', 'Color', color_input, 'linewidth', 2) ;

    
