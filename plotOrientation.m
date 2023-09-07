function h = plotOrientation(location, scaling, theta)
    
     Rz = [cos(theta)    sin(theta) 0
           -sin(theta)   cos(theta)]
     
     %get X axis
     vec = Rz(:,1)*scaling;
     tip = location + vec;
     h = line([location(1) tip(1) location(2) tip(2)])
     
end