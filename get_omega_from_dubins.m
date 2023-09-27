
function omegas = get_omega_from_dubins(dubins_obj, v, rho)
    omega = v / rho;
    dirs = cellfun(@(dir) get_turning_sign_from_dubins_direction(dir), dubins_obj.MotionTypes);
    omegas = omega * dirs;
end
