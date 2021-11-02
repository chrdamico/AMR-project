function LIPdynamics(solv)
    % Simply implements the dynamic model of a LIP with Euler
    % integration
    dcm_dot = solv.omega*(solv.dcm_pos-solv.vrp);
    solv.dcm_pos = solv.dcm_pos + solv.dT*dcm_dot;
end