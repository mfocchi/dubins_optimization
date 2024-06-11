function [omega_min, omega_max] = computeFeasibleOmega(v)
    constants.GEARBOX = 34.45;
    constants.TRACK_WIDTH = 0.606;
    constants.SPROCKET_RADIUS = 0.0856;
    constants.WHEEL_MAX = 160;
    
    v_nom = 0.2;
    
    C = [-constants.SPROCKET_RADIUS/(constants.TRACK_WIDTH*constants.GEARBOX); constants.SPROCKET_RADIUS/(constants.TRACK_WIDTH*constants.GEARBOX)];
    Aeq = [constants.SPROCKET_RADIUS/(2*constants.GEARBOX ), constants.SPROCKET_RADIUS/(2*constants.GEARBOX )];
    beq = v_nom;
    lb = [-constants.WHEEL_MAX, -constants.WHEEL_MAX];
    ub = [constants.WHEEL_MAX, constants.WHEEL_MAX];
    [x, feval] = linprog(C,[],[],Aeq,beq,lb,ub);
    omega_max = -feval;
    [x, feval] = linprog(-C,[],[],Aeq,beq,lb,ub);
    omega_min = feval;
end

