function getDCMeosAndZMP(plan)
    % Initialize quantities
    plan.zmp_traj_store=plan.f_pos_des;
    plan.dcm_traj_des_eos_store(:,plan.num_steps)=plan.zmp_traj_store(1:2,plan.num_steps);
    plan.dcm_traj_des_eos_store(:,1) = plan.f_pos_des(1:2,1);
    % Use Romuladi's eq. 5 to get EndOfStep values of DCM
    for i=plan.num_steps-1:-1:plan.current_foot+1
        tstepi=plan.t_imp_des(i+1)-plan.t_imp_des(i);
        plan.dcm_traj_des_eos_store(:,i)=plan.zmp_traj_store(1:2,i)+exp(-plan.omega*tstepi)*(plan.dcm_traj_des_eos_store(:,i+1)-plan.zmp_traj_store(1:2,i));
    end
%     plan.dcm_traj_des_eos_store(:,1)=[0;0];
%     plan.dcm_traj_des_eos_store(:,1)=plan.zmp_traj_store(1:2,1);
end