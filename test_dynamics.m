close all 
clear all 



%WORLD FRAME ATTACHED TO ANCHOR 1

params.int_method = 'rk4';
params.int_steps = 5;
N_dyn = 100;


%initial state
x0 =  [0.;0.; 0.3];
dt = 0.01;
Tf =  10;
dt_dyn = Tf / (N_dyn-1);
%inputs
n_sim_steps = floor(Tf/dt);

u_omega_l = 10;
u_omega_r = 13;

omega_l = ones(1, n_sim_steps) * u_omega_l;
omega_r = ones(1, n_sim_steps) * u_omega_r;

params.width = 0.606; % [m]
params.sprocket_radius = 0.0979; % [m]
params.gearbox_ratio = 39.4;
params.slip_fit_coeff.left  = [-0.0591   -0.2988];
params.slip_fit_coeff.right = [0.0390    0.2499 ];


[~,~,x, t] = integrate_dynamics(x0, 0, dt, n_sim_steps, omega_l, omega_r, params);


final_point = [x(1,end), x(2,end), x(3,end)];
ode45_final_point =[ -0.0000,    1.6240,   -1.6246];
fprintf('Compare with ode45 final point [%3.4f, %3.4f, %3.4f] \n', final_point- ode45_final_point)



% figure
% subplot(3,1,1)
% plot(t, X ,'-ro'); 
% grid on;
% subplot(3,1,2)
% plot(t, Y ,'-ro');
% grid on;
% subplot(3,1,3)
% plot(t, Z ,'-ro');


% integrate with substeps (every Ndyn/int_steps)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

omega_l_rough = ones(1, N_dyn) * u_omega_l;
omega_r_rough = ones(1, N_dyn) * u_omega_r;
[states_rough_sub, t_rough_sub] = computeRollout(x0, 0,dt_dyn, N_dyn, omega_l_rough, omega_r_rough, params);
 
% integrate without substep (every Ndyn) 
params.int_steps = 0;
[states_rough, t_rough] = computeRollout(x0, 0,dt_dyn, N_dyn, omega_l_rough, omega_r_rough, params);


figure
subplot(3,1,1)
plot(t_rough, states_rough(1,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(1,:),'ko'); hold on;grid on;
plot(t, x(1,:),'-r'); 
legend('rough','rough sub','cont');
ylabel('X')

subplot(3,1,2)
plot(t_rough, states_rough(2,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(2,:),'ko'); hold on;grid on;
plot(t, x(2,:),'-r');
ylabel('Y')

subplot(3,1,3)
plot(t_rough, states_rough(3,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(3,:),'ko'); hold on; grid on;
plot(t, x(3,:),'-r');
ylabel('theta')


