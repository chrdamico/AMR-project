function planNominalTrajectory(plan, f_pos_current, t_current, left_current, current_foot)

% Update current foot position boolean
plan.left = left_current;
% Compute start position for the unicycle 
plan.current_foot = current_foot;
% Initialize new initial position of the unicycle based on the current
% starting feet position
if plan.left
    displacement = [0; +plan.m];
else  
    displacement = [0; -plan.m];
end

% Compute foot position
R = plan.rotateAroundZ(f_pos_current(3));
R = R(1:2,1:2);

position = f_pos_current(1:2) + R*displacement;
x_start = [position; f_pos_current(3)];
T_current = t_current;

% Sample a unicycle from start to goal
plan.sampleUnicycle(x_start, plan.x_goal, T_current)

% Generate num_steps footsteps from the sampled trajectory using Dafarra's
% optimization method
plan.genNominalFootsteps(f_pos_current, T_current, current_foot)

% Update the plan ensuring that the previous foot position is the correct
% one that is in solv
plan.f_pos_des(:, plan.current_foot) = f_pos_current;

% Get position of DCMeos and ZMP
plan.getDCMeosAndZMP()

end