clear all;
global params 

addpath("../");

%https://it.mathworks.com/help/ros/ug/call-and-provide-ros2-services.html

folderPath =  fullfile(pwd,"custom");

% do only once
% ros2genmsg(CreateShareableFile=true)
% clear classes
% hash toolboxcache

%TODO
%addpath('find where is this foder/glnxa64/install/m')


node_1 = ros2node("node_1");
node_2 = ros2node("node_2");

run('robot_params.m');

server = ros2svcserver(node_1,"/optim","custom/Optim",@OptimCallback);
%client
p0 = [0.0; 0.0; -0.]; 
pf = [-0.4758; -1.1238; 0.9638];

client = ros2svcclient(node_2,"/optim","custom/Optim");
req = ros2message(client);

req.x0 = p0(1);
req.y0 = p0(2);
req.theta0 = p0(3);

req.xf= pf(1);
req.yf = pf(2);
req.thetaf = pf(3);

numCallFailures = 0;
[resp,status,statustext] = call(client,req,"Timeout",3);
if ~status
    numCallFailures = numCallFailures + 1;
    fprintf("Call failure number %d. Error cause: %s\n",numCallFailures,statustext)
else
    disp(resp)
end

% resp.des_x'
% resp.des_y'
% resp.des_theta'


