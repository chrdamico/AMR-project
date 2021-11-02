% Load parameters
loadParameters

% Initialize planner
plan = Planner(plan_pars);
% Initialize solver
solv = Solver(pars);

% Plan initial trajectory and initialize solver quantities with it
plan.planNominalTrajectory(plan.f_pos_des(:,1), plan.t_imp_des(1) + plan.dT, false, 1);
% Load planner results in solver
solv.f_pos_des = plan.f_pos_des; % Feet position plan
solv.dcm_traj_des_eos_store = plan.dcm_traj_des_eos_store; % DCM at the end of a step plan
solv.t_imp_des = plan.t_imp_des; % Impact times plan
original_times = plan.t_imp_des; % saving the original times plan for a plot

% Set initial DCM position
solv.dcm_pos = solv.f_pos_des(1:2,1);
solv.dcm_vel = [0;0];

% Main simulation loop
iter = 0;
done = false;
while not(done)
    iter = iter + 1;
    solv.t_curr = iter*solv.dT;
    
    % Check for completion of a step
    if solv.t_curr > solv.t_imp_des(solv.f_iter)
        
        % Replan the trajectory if necessary
        if solv.replan
            disp('replanning, step:')
            disp(solv.f_iter)
            % initializing planner:
            plan.dcm_traj_des_eos_store = solv.dcm_traj_des_eos_store;
            plan.planNominalTrajectory(solv.f_pos_des(:,solv.f_iter), solv.t_imp_des(solv.f_iter), solv.left, solv.f_iter);
            % Load planner results in solver
            solv.f_pos_des = plan.f_pos_des; % Feet position
            solv.dcm_traj_des_eos_store = plan.dcm_traj_des_eos_store; % DCM at the end of a step 
            solv.t_imp_des = plan.t_imp_des; % Impact times
            solv.replan = false;
        else
            disp('step:')
            disp(solv.f_iter)
        end
        
        % Update current footstep flag
        if solv.left
            solv.left = false;
        else
            solv.left = true;
        end

        
        % Update footstep counter number
        solv.f_iter = solv.f_iter + 1;
    end
    
    
     % termination condition based on number of steps
    if solv.f_iter>solv.num_steps
        done = true;
        break
    end    

    % Cycle smoothing, dynamics and control loop
    solv.cycle(iter);
  

% figure(1),clf
% hold on 
% grid on
% plot(solv.quadprog_sol_store(1,1:iter), solv.quadprog_sol_store(2,1:iter), 'b.')
% plot(solv.f_pos_des(1,:),solv.f_pos_des(2,:), 'r')
% title("Quadprog solution compared to ZMP desired", 'interpreter', 'latex', 'FontSize', 14)
% legend('quadprog','$r^{ZMP}$', 'interpreter', 'latex', 'FontSize', 14,'Location','southeast')
% xlabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
% ylabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
% xlim([0.5,4])
% ylim([0.6, 1.3])
% pbaspect([2,1,1])
% drawnow
% hold off
% % 
% figure(2),clf
% hold on
% grid on
% plot(solv.dcm_traj_des_eos_store(1,1:end),solv.dcm_traj_des_eos_store(2,1:end), 'b')
% plot(solv.quadprog_sol_store(3,1:iter), solv.quadprog_sol_store(4,1:iter), 'r.')
% xlabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
% ylabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
% title("DCMeos from step adapter compared to DCMeos planned", 'interpreter', 'latex', 'FontSize', 14)
% legend('DCMeos planned','DCM eos', 'interpreter', 'latex', 'FontSize', 14,'Location','southeast')
% xlim([0.5,4])
% ylim([0.6, 1.3])
% pbaspect([2,1,1])
% hold off
end



figure(1),clf
hold on 
grid on
plot(plan.f_pos_des(1,1:end-1),plan.f_pos_des(2,1:end-1), 'r')
plot(solv.dcm_traj_des_store(1,1:iter-1),solv.dcm_traj_des_store(2,1:iter-1), 'b')
% Rectangles for the steps
ll = solv.f_pos_des(1:2,1) - [0;2*solv.m] - [0.04; 0.04];
rel_ur = [0.08; 0.08];
rectangle('position', [ll(1), ll(2), rel_ur(1), rel_ur(2)]);
for i=1:solv.num_steps
    ll = solv.f_pos_des(1:2,i) - [0.04; 0.04];
    rel_ur = [0.08; 0.08];
    rectangle('position', [ll(1), ll(2), rel_ur(1), rel_ur(2)]);
end
title("Desired ZMP and DCM trajectories", 'interpreter', 'latex', 'FontSize', 14)
legend('$r^{ZMP}$','$\xi$', 'interpreter', 'latex', 'FontSize', 14,'Location','southeast')
xlabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
ylabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
xlim([0.5,4])
ylim([0.6, 1.3])
pbaspect([2,1,1])
hold off


figure(2),clf
hold on 
grid on
plot(solv.dcm_traj_store(1,:),solv.dcm_traj_store(2,:), 'b')
plot(solv.vrp_des_store(1,:), solv.vrp_des_store(2,:), 'r')
% Rectangles for the steps 
ll = solv.f_pos_des(1:2,1) - [0;2*solv.m] - [0.04; 0.04];
rel_ur = [0.08; 0.08];
rectangle('position', [ll(1), ll(2), rel_ur(1), rel_ur(2)]);
for i=1:solv.num_steps
    ll = solv.f_pos_des(1:2,i) - [0.04; 0.04];
    rel_ur = [0.08; 0.08];
    rectangle('position', [ll(1), ll(2), rel_ur(1), rel_ur(2)]);
end
title("Actual DCM and VRP trajectories", 'interpreter', 'latex', 'FontSize', 14)
legend('$\xi$', '$vrp$', 'interpreter', 'latex', 'FontSize', 14,'Location','southeast')
xlabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
ylabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
xlim([0.5,4])
ylim([0.6, 1.3])
pbaspect([2,1,1])
hold off

figure(3),clf
hold on
grid on
plot(solv.dcm_traj_store(1,1:end),solv.dcm_traj_store(2,1:end), 'b')
plot(solv.dcm_traj_des_store(1,1:iter-1), solv.dcm_traj_des_store(2,1:iter-1), 'g')
xlabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
ylabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
title("DCM trajectory compared to desired value", 'interpreter', 'latex', 'FontSize', 14)
legend('DCM ','DCM desired', 'interpreter', 'latex', 'FontSize', 14,'Location','southeast')
xlim([0.5,4])
ylim([0.6, 1.3])
pbaspect([2,1,1])
hold off


% StepAdapter
figure(4),clf
hold on 
grid on
plot(solv.quadprog_sol_store(1,1:iter), solv.quadprog_sol_store(2,1:iter), 'b.')
plot(solv.f_pos_des(1,:),solv.f_pos_des(2,:), 'r')
% Rectangles for the steps 
ll = solv.f_pos_des(1:2,1) - [0;2*solv.m] - [0.04; 0.04];
rel_ur = [0.08; 0.08];
rectangle('position', [ll(1), ll(2), rel_ur(1), rel_ur(2)]);
for i=1:solv.num_steps
    ll = solv.f_pos_des(1:2,i) - [0.04; 0.04];
    rel_ur = [0.08; 0.08];
    rectangle('position', [ll(1), ll(2), rel_ur(1), rel_ur(2)]);
end
title("Quadprog solution compared to ZMP desired", 'interpreter', 'latex', 'FontSize', 14)
legend('quadprog','$r^{ZMP}$', 'interpreter', 'latex', 'FontSize', 14,'Location','southeast')
xlabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
ylabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
xlim([0.5,4])
ylim([0.6, 1.3])
pbaspect([2,1,1])
hold off

figure(5),clf
hold on 
grid on
t = linspace(0, solv.dT*iter, iter-1);
time_difference = original_times - solv.t_imp_des;
plot(diff(time_difference))
title("Modification of impact time from stepAdapter", 'interpreter', 'latex', 'FontSize', 14)
legend('$\Delta t$', 'interpreter', 'latex', 'FontSize', 14,'Location','southeast')
xlabel("Step number", 'interpreter', 'latex', 'FontSize', 14)
ylabel("time variation [s]", 'interpreter', 'latex', 'FontSize', 14)
pbaspect([2,1,1])
hold off

figure(6),clf
hold on
grid on
plot(solv.dcm_traj_des_eos_store(1,1:end),solv.dcm_traj_des_eos_store(2,1:end), 'b')
plot(solv.quadprog_sol_store(3,1:iter), solv.quadprog_sol_store(4,1:iter), 'r.')
xlabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
ylabel("Position in space [m]", 'interpreter', 'latex', 'FontSize', 14)
title("DCMeos from step adapter compared to DCMeos planned", 'interpreter', 'latex', 'FontSize', 14)
legend('DCMeos planned','DCM eos', 'interpreter', 'latex', 'FontSize', 14,'Location','southeast')
xlim([0.5,4])
ylim([0.6, 1.3])
pbaspect([2,1,1])
hold off



