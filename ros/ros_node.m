clc;clear all;close all;
rosshutdown
rosinit
global params 

addpath("../");

%https://it.mathworks.com/help/ros/ug/call-and-provide-ros2-services.html
homedir = getenv('HOME');



% do only once
% cd('../') % you need to move to the parent folder that
% contains the optim_interfaces package with the srv folder
% rmdir('matlab_msg_gen','s') % removes the previous folder
% rosgenmsg / ros2genmsg % this creates the new  matlab_msg_gen folder inside  lyapunov_slippage_controller, (CreateShareableFile= true ) creates the zip
% clear classes
% rehash toolboxcache
%


type_of_ros = 'ros1';

if strcmp(type_of_ros,'ros2')
    disp('ros2')
    addpath('../matlab_msg_gen/glnxa64/install/m')
    node_1 = ros2node("optim_server");
    node_2 = ros2node("optim_client");
else
  
    addpath('../matlab_msg_gen_ros1/glnxa64/install/m')
    masterHost = 'localhost';
    node_1 = ros.Node('optim_server', masterHost);
    node_2 = ros.Node('optim_client', masterHost);
end

%THESE ARE THE SETTINGS THE ROBOT WILL USE!
run('robot_params.m');

params.dt=0.01; % in matlab is used only to evaluate solution, but it will be sent to c++ to define the discretization for the path generation

if ~isfile('../optimize_cpp_mex.mexa64')
    disp('Generating C++ code');
    cfg = coder.config('mex');
    cfg.IntegrityChecks = false;
    cfg.SaturateOnIntegerOverflow = false;
    codegen -config cfg  optimize_cpp -args {zeros(3,1), zeros(3,1),coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]),0, coder.cstructname(params, 'param') } -nargout 1 -report
    movefile('optimize_cpp_mex.mexa64','../');
end


if strcmp(type_of_ros,'ros2')
    server = ros2svcserver(node_1,"/optim","optim_interfaces/Optim",@OptimCallbackRos2);
else
    server = rossvcserver('/optim', 'optim_interfaces/Optim', @OptimCallbackRos1,'DataFormat','struct');
end

%client
% INITIAL STATE (X,Y, THETA)
p0 = [0.0; 0.0; -0.]; 
%FINAL STATE  (X,Y, THETA)
pf = [2.; 2.5; 0.];
plan_type = 'dubins'; % 'optim' 'dubins'

if strcmp(type_of_ros,'ros2')
    client = ros2svcclient(node_2,"/optim","optim_interfaces/Optim");
    req = ros2message(client);
    req.x0 = p0(1);
    req.y0 = p0(2);
    req.theta0 = p0(3);
    
    req.xf= pf(1);
    req.yf = pf(2);
    req.thetaf = pf(3);
    req.plan_type=plan_type;


    numCallFailures = 0;
    [resp,status,statustext] = call(client,req,"Timeout",10);
    if ~status
        numCallFailures = numCallFailures + 1;
        fprintf("Call failure number %d. Error cause: %s\n",numCallFailures,statustext);
    end

      disp(length(resp.des_x))
    resp.des_x'
    resp.des_y'
    resp.des_theta'
    resp.des_v'
    resp.des_omega'
    resp.dt
else

    client = rossvcclient("/optim","DataFormat","struct");
    req = rosmessage(client);
    req.X0 = p0(1);
    req.Y0 = p0(2);
    req.Theta0 = p0(3);
    
    req.Xf= pf(1);
    req.Yf = pf(2);
    req.Thetaf = pf(3);
    req.PlanType=plan_type;



    if isServerAvailable(client)
        tic
        resp = call(client,req, "Timeout",5)
        toc
    else
        error("Service server not available on network")
    end

    disp(length(resp.DesX))
    resp.DesX(1:20)'
    resp.DesY(1:20)'
    resp.DesTheta(1:20)'
    resp.DesV(1:20)'
    resp.DesOmega(1:20)'
    resp.Dt

end


