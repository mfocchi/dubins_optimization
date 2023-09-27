function [omega_l0, omega_r0, t_rough] = getVelocityParamsFromDubin(params, lengths, omegas) 
    % get total time 
    tf = sum(lengths)/params.v_max; 
    % build up velocity trajectories
    sample_instants = round(lengths/sum(lengths)*params.N_dyn);
    omega0 = 0.*ones(1,params.N_dyn);
    v0 = params.v_max.*ones(1,params.N_dyn);
    idx1 = sample_instants(1);
    idx2 = sample_instants(1)+sample_instants(2);
    idx3 = sum(sample_instants);
    omega0(1:idx1-1) = omegas(1);
    omega0(idx1:idx2-1) = omegas(2);
    omega0(idx2:idx3) = omegas(3);
    r = params.sprocket_radius;
    B = params.width;
    A = [r/2, r/2; r/B, -r/B];
    
    t_rough = [];
    t_= 0.;
    dt = tf / (params.N_dyn );
    for i=1:params.N_dyn
      y = inv(A)*[v0(i); omega0(i)];
      omega_r0(i) = params.gearbox_ratio*y(1); 
      omega_l0(i) = params.gearbox_ratio* y(2);
      t_ = t_ + dt;
      t_rough = [t_rough t_];
    end
    
end
