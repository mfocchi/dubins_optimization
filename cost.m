function cost = cost(x, p0,  pf, params)

    Tf =  x(1);
    omega_l = x(params.num_params+1:params.num_params+params.N_dyn); 
    omega_r = x(params.num_params+params.N_dyn+1:params.num_params+2*params.N_dyn); 

    
    
    % check they are column vectors
    p0 = p0(:);
    pf = pf(:);

    dt_dyn = Tf / (params.N_dyn-1); 
    
    %single shooting
    [states, t] = computeRollout(p0, 0,dt_dyn, params.N_dyn, omega_l, omega_r, params);
    x = states(1,:);
    y = states(2,:);
    theta = states(3,:);
    
    p_0 = [x(:,1); y(:,1); theta(:,1)];
    p_f = [x(:,end); y(:,end); theta(:,end)];


    % be careful there are only N values in this vector the path migh be
    % underestimated!
%     deltax = diff(p(1,:));  % diff(X);
%     deltay = diff(p(2,:));   % diff(Y);
%     deltaz = diff(p(3,:));    % diff(Z);
%     path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2));

    [v_input,omega_input] =  computeVelocitiesFromTracks(omega_l, omega_r, params);
    tracking = norm(p_f - pf);
    %smoothing = sum(diff(omega_l).^2)+ sum(diff(omega_l).^2);  
    smoothing = sum(diff(v_input).^2)+ sum(diff(omega_input).^2); 
       
    cost =    params.w1 * Tf + params.w2 *smoothing  + params.w3 *tracking;
end