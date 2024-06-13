clc;clear all;close all;
rosshutdown
rosinit
global params 

addpath("../");

%https://it.mathworks.com/help/ros/ug/call-and-provide-ros2-services.html
homedir = getenv('HOME');

%custom_message_folder = strcat(homedir,'/ros2_docker/ros2_ws/src/lyapunov_slippage_controller/');
%addpath(custom_message_folder)

% do only once
% cd('../') % you need to move to the parent folder that
% contains the optim_interfaces package with the srv folder
% rmdir('matlab_msg_gen','s') % removes the previous folder
% ros2genmsg % this creates the new  matlab_msg_gen folder inside  lyapunov_slippage_controller, (CreateShareableFile= true ) creates the zip
% clear classes
% rehash toolboxcache


node_1 = ros2node("optim_server");
node_2 = ros2node("optim_client");

%THESE ARE THE SETTINGS THE ROBOT WILL USE!
run('robot_params.m');

params.dt=0.01; % in matlab is used only to evaluate solution, but it will be sent to c++ to define the discretization for the path generation

if ~isfile('../optimize_cpp_mex.mexa64')
    disp('Generating C++ code');
    cfg = coder.config('mex');
    cfg.IntegrityChecks = false;
    cfg.SaturateOnIntegerOverflow = false;
    codegen -config cfg  optimize_cpp -args {zeros(3,1), zeros(3,1),coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]),0, coder.cstructname(params, 'param') } -nargout 1 -report
end


server = ros2svcserver(node_1,"/optim","optim_interfaces/Optim",@OptimCallback);
%client
% INITIAL STATE (X,Y, THETA)
p0 = [0.0; 0.0; -0.]; 
%FINAL STATE  (X,Y, THETA)
pf = [1.5; 2.1238; pi/2];


client = ros2svcclient(node_2,"/optim","optim_interfaces/Optim");
req = ros2message(client);

req.x0 = p0(1);
req.y0 = p0(2);
req.theta0 = p0(3);

req.xf= pf(1);
req.yf = pf(2);
req.thetaf = pf(3);

req.plan_type='dubins'; % 'optim' 'dubins'

numCallFailures = 0;
[resp,status,statustext] = call(client,req,"Timeout",10);
if ~status
    numCallFailures = numCallFailures + 1;
    fprintf("Call failure number %d. Error cause: %s\n",numCallFailures,statustext);
end

%disp(length(resp.des_x))
% resp.des_x'
% resp.des_y'
% resp.des_theta'
% resp.des_v'
% resp.des_omega'
%resp.dt
