function resp = OptimCallback(~,req,resp)
global params

p0(1) = req.P0.X;
p0(2) = req.P0.Y;
p0(3) = req.P0.Z;

pf(1) = req.Pf.X;
pf(2) = req.Pf.Y;
pf(3) = req.Pf.Z;

dubConnObj = dubinsConnection;
curvature_max = params.omega_max/params.v_max;
dubConnObj.MinTurningRadius = 1/curvature_max;
[pathSegObj, pathCosts] = connect(dubConnObj,p0',pf');
%show(pathSegObj{1})
% get total time
t0 = sum(pathSegObj{1}.MotionLengths)/params.v_max;
% compute the omega from dubin
omegas = get_omega_from_dubins(pathSegObj{1}, params.v_max, 1/curvature_max);
%map to wheel omega
[omega_l0, omega_r0, t_rough] = getVelocityParamsFromDubin(params, pathSegObj{1}.MotionLengths, omegas);

solution = optimize_cpp_mex(p0,  pf, omega_l0, omega_r0, t0,  params); 
resp.LinVel.Data = solution.v(1);
resp.AngVel.Data = solution.omega(1);
 

end