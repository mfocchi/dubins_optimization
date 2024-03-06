% Reset environment
close all; clear all; clc;

addpath("../");
x0 = 0; 
y0 = 0; 
th0 = -2/3*pi;
xf = 4; 
yf = 0; 
thf = pi/3.0;
Kmax = 3.0;

dubinsObj = dubinsClass;
% Find optimal Dubins solution
[pidx, curve] = dubinsObj.dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax);

% Plot and display solution if valid
if pidx > 0
  figure; axis equal;
  dubinsObj.plotdubins(curve, true, [1 0 0], [0 0 0], [1 0 0]);
  curve.a1
  curve.a2
  curve.a3
  curve.L
else
  fprintf('Failed!\n');
end