clear all; close all ;

addpath("../");

p0 = [0.0 0.0 -0.3]; 
pf = [1.5 1.0 -1.2];



params.int_method = 'rk4';
params.width = 0.606; % [m]
params.sprocket_radius = 0.0979; % [m]
params.gearbox_ratio = 39.4;

params.model = 'UNICYCLE';
params.N_dyn = 40;
params.omega_max = 5.5;
params.v_max = 1;
curvature_max = params.omega_max/params.v_max;

% p0 = [0.0 0.0 -0.0]; 
% pf = [-0.4758 -1.1238 1.1638];
% params.omega_max = 1;
% params.v_max = 0.1;
% curvature_max = params.omega_max/params.v_max;


%LUIGI
grid on;
dubinsObj= dubinsClass;
% Find optimal Dubins solution
[pidx, curve, curvatures, lenghts] = dubinsObj.dubins_shortest_path(p0(1), p0(2), p0(3), pf(1), pf(2), pf(3), curvature_max);
%plot dubin
figure(1)
title('LUIGI dubin')
dubinsObj.plotdubins(curve, true, [1 0 0], [0 0 0], [1 0 0])


%matlab 
dubConnObj = dubinsConnection;
dubConnObj.MinTurningRadius = 1/curvature_max;
[pathSegObj, pathCosts] = connect(dubConnObj,p0,pf);
figure(2)
show(pathSegObj{1})
title('MATLAB dubin')



% get total time 
t0 = sum(pathSegObj{1}.MotionLengths)/params.v_max; 


%get discrete omegas (3 elements)
omegas = get_omega_from_dubins(pathSegObj{1}, params.v_max, 1/curvature_max);
switch_times = [cumsum(pathSegObj{1}.MotionLengths) / params.v_max];


% build fine vector
dt = 0.001; %with rougher value you will see some difference (e.g.  0,1)
omega0 = [];
v0 = [];
omega_r0 = [];
omega_l0 = [];
t_ = 0.;
time = [];
while (t_<=t0)
    if  t_ < switch_times(1)
        omega =  omegas(1);
    elseif t_ <switch_times(2)
        omega = omegas(2);
    else 
        omega = omegas(3);
    end
    omega0 = [omega0 omega];
    v0 = [ v0 params.v_max];
    
    r = params.sprocket_radius;
    B = params.width;
    A = [r/2, r/2; r/B, -r/B];
    y = inv(A)*[params.v_max; omega];
    omega_r0 = [omega_r0 params.gearbox_ratio*y(1)]; 
    omega_l0 = [omega_l0 params.gearbox_ratio* y(2)];
    time = [time t_ ];
    t_ =  t_+ dt;
end

params.int_steps = 0;
%integrate the fine grid omegas
[states, t] = computeRollout(p0, 0,dt, length(omega_l0), omega_l0, omega_r0, params);

%compare with the rough one
% dt = t0 / (params.N_dyn )
% [omega_rough_l0, omega_rough_r0, time_rough] = getVelocityParamsFromDubin(params, pathSegObj{1}.MotionLengths, omegas);
% params.int_steps = 0;
% [states, t] = computeRollout(p0, 0,dt, params.N_dyn, omega_rough_l0, omega_rough_r0, params);

x = states(1,:);
y = states(2,:);
theta = states(3,:);

p =  [x; y; theta];
figure
plot(pf(1), pf(2) , 'Marker', '.', 'Color','r', 'MarkerSize',60) ;
min_x = min(min(p(1,:)), pf(1))-3 ;
max_x = max(max(p(1,:)), pf(1))+3 ;
min_y = min(min(p(2,:)), pf(2))-3 ;
max_y = max(max(p(2,:)), pf(2)) +3 ;

% set(gca,'XLim',[min_x max_x])
% set(gca,'YLim',[min_y max_y])

color_input = 'r'; 

% actual traj obtained by fine integration
plot(p(1,:), p(2,:), 'o', 'Color', color_input ) ;
% 
% 
scaling = 0.1*norm(pf(1:2) - p0(1:2));
%initial orient
plotOrientation([p0(1); p0(2)], p0(3), scaling);
%final orient 
plotOrientation([pf(1); pf(2)], pf(3), 0.1);
grid on;hold on;
dubinsObj.plotdubins(curve, true, [1 0 0], [0 0 0], [1 0 0])
xlabel('X');
ylabel('Y');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%figure 
figure
% on a fine grid
plot(time, omega_l0);   hold on;
% compute on a rough grid Ndyn
[omega_rough_l0, omega_rough_r0, time_rough] = getVelocityParamsFromDubin(params, pathSegObj{1}.MotionLengths, omegas);
plot(time_rough, omega_rough_l0, 'o');   hold on; grid on;
title("compare rough with fine grid")
legend('fine', 'rough')

