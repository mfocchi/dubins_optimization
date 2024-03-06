clear all;
global params

addpath("../");

%https://it.mathworks.com/help/ros/ug/call-and-provide-ros2-services.html

folderPath =  fullfile(pwd,"custom");

% do only once
ros2genmsg(folderPath, CreateShareableFile=true)
clear classes
hash toolboxcache

%TODO
%addpath('find where is this foder/glnxa64/install/m')


node_1 = ros2node("node_1");
node_2 = ros2node("node_2");

run('robot_params.m');

server = ros2svcserver(node_1,"/mpc","customMessages/mpc",@OptimCallback, 'DataFormat','struct')             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%client
p0 = [0.0; 0.0; -0.]; 
pf = [-0.4758; -1.1238; 0.9638];

client = ros2svcclient(node_2,"/mpc","customMessages/mpc","DataFormat","struct")
req = ros2message(client);

req.x0 = p0(1);
req.y0 = p0(2);
req.theta0 = p0(3);

req.xf= pf(1);
req.xf = pf(2);
req.thetaf = pf(3);


if isServerAvailable(mpcclient)
    tic
    resp = call(client,req,"Timeout",3);
    toc
else
    error("Service server not available on network")
end


for k = 1:params.mpc_N
    des_x(k) = resp.DesX{k}.Data;
    des_y(k) = resp.DesX{k}.Data;
    des_theta(k) = resp.DesX{k}.Data;   
end
% 
