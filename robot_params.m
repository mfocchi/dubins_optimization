


params.int_method = 'rk4'; %'eul'  'rk4';
params.N_dyn = 40; %dynamic constraints (number of knowts in the discretization) 
params.int_steps = 10 ;%cast(5,"int64"); %0 means normal intergation
params.num_params = 1; %final time

params.w1= 1;  % minimum time (fundamental!)
params.w2= 1; % smoothing on omega, v  
params.w3= 0; % smoothing on xy_der
params.w4= 0; %smoothing on theta der 

tau_gearbox = 34.45;
%params.model = 'UNICYCLE';
params.model = 'SIDEONLY';
%params.model = 'LONGSIDE';
% max rpm for the motor = 1500, selecting some value below
RPM2RADS = 1/60*2*pi;
params.omega_w_max = 1500 *RPM2RADS/ tau_gearbox; % the decision variables are the wheel of the unycicle not the motors, these are 4.5 rad/s
                                                  % these correspond to
                                                  % v_max = 0.42 and
                                                  % omega_max =1.4
                                               
params.omega_max = 0.2;
params.omega_min = -0.2;
params.v_max = 0.1;
params.v_min = 0.01; % if you want optimal control similar to dubins put vmax = vmin and use unicycle model
params.VELOCITY_LIMITS = true;
params.t_max = 80; %TODO put a check on this
params.slack_target = 0.02;
params.DEBUG_COST = false;
constr_tolerance = 1e-3;
params.dt=0.001; % in matlab is used only to evaluate solution, but it will be sent to c++ to define the discretization for the path generation

params.width = 0.606; % [m]
params.sprocket_radius = 0.0856; % [m]

params.side_slip_angle_coefficients_left = [-0.3795,   -3.3784];
params.side_slip_angle_coefficients_right = [0.4587,    3.8471];
params.beta_slip_inner_coefficients_left = [ -0.0579,   -2.4456];
params.beta_slip_outer_coefficients_left =  [  0.0588 ,  -2.6375];
params.beta_slip_inner_coefficients_right = [ -0.0618 ,   3.0089];
params.beta_slip_outer_coefficients_right = [  0.0906,    3.7924];



