function vrp_des = simplifiedModelControl(solv, iter)
    % Compute the desired vrp control from eq. 14 of Shafiee

    vrp_des = solv.dcm_traj_des_store(:,iter) - (1/solv.omega)*solv.dcm_vel_des_store(:,iter) + ...
        solv.Kxi*(solv.dcm_pos - solv.dcm_traj_des_store(:,iter));

%     traj_des=solv.dcm_traj_des_store(:,iter)
%     val_des=solv.dcm_vel_des_store(:,iter)
%     traj=solv.dcm_pos
end