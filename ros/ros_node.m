clc;clear all;close all;
rosshutdown
rosinit
global params 

addpath("../");

%https://it.mathworks.com/help/ros/ug/call-and-provide-ros2-services.html

% do only once
%cd('/home/laboratorio/ros2_docker/ros2_ws/src/')
% clear classes
% ros2genmsg % this creates the zip (CreateShareableFile=true)
% rehash toolboxcache


addpath('/home/laboratorio/ros2_docker/ros2_ws/src/matlab_msg_gen/')


node_1 = ros2node("optim_server");
node_2 = ros2node("optim_client");

run('robot_params.m');

server = ros2svcserver(node_1,"/optim","optim_interfaces/Optim",@OptimCallback);
%client
p0 = [0.0; 0.0; -0.]; 
pf = [-0.4758; -1.3; 0.9638];

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

resp.des_x'
resp.des_y'
resp.des_theta'


