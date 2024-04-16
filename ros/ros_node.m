clc;clear all;close all;
rosshutdown
rosinit
global params 

addpath("../");

%https://it.mathworks.com/help/ros/ug/call-and-provide-ros2-services.html

% do only once
% cd('/home/laboratorio/ros2_docker/ros2_ws/src/') 
% ros2genmsg % this creates the zip (CreateShareableFile=true)
% clear classes
% rehash toolboxcache


addpath('/home/laboratorio/ros2_docker/ros2_ws/src/matlab_msg_gen/')
node_1 = ros2node("optim_server");
node_2 = ros2node("optim_client");

%THESE ARE THE SETTINGS THE ROBOT WILL USE!
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
                                               
params.omega_max = 1.5;
params.omega_min = -1.5;
params.v_max = 0.5;
params.v_min = 0.1;
params.VELOCITY_LIMITS = true;
params.t_max = 80; %TODO put a check on this
params.slack_target = 0.02;
params.DEBUG_COST = false;
constr_tolerance = 1e-3;
params.dt=0.01; % only to evaluate solution it will be used in the path generation

params.width = 0.606; % [m]
params.sprocket_radius = 0.0856; % [m]
params.slip_fit_coeff.left  = [-0.0591,   -0.2988];
params.slip_fit_coeff.right = [0.0390,    0.2499 ];
params.side_slip_fit_coeff = [-0.7758   -6.5765];
params.slip_fit_coeff.min_value = -1000;
params.locked_wheel_coeff = [0 1]


server = ros2svcserver(node_1,"/optim","optim_interfaces/Optim",@OptimCallback);
%client
% INITIAL STATE (X,Y, THETA)
p0 = [0.0; 0.0; -0.]; 
%FINAL STATE  (X,Y, THETA)
pf = [0.5; -1.1238; 0.3638];


client = ros2svcclient(node_2,"/optim","optim_interfaces/Optim");
req = ros2message(client);

req.x0 = p0(1);
req.y0 = p0(2);
req.theta0 = p0(3);

req.xf= pf(1);
req.yf = pf(2);
req.thetaf = pf(3);

req.plan_type='optim'; % 'optim' 'dubins'

numCallFailures = 0;
[resp,status,statustext] = call(client,req,"Timeout",10);
if ~status
    numCallFailures = numCallFailures + 1;
    fprintf("Call failure number %d. Error cause: %s\n",numCallFailures,statustext);
end

% resp.des_x'
% resp.des_y'
% resp.des_theta'
% resp.des_v'
% resp.des_omega'
% resp.dt
