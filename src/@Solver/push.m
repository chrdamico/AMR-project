function push(solv,iter)
    if iter>1 
        % Push
        f=50*[1,0]'; %[N]
        % Euler integration
        dcm_dot = solv.omega*(solv.dcm_pos-solv.vrp) + f/(solv.omega*solv.mass);
        solv.dcm_pos = solv.dcm_pos + solv.dT*dcm_dot;   
    end
end