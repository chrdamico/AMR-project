function genNominalFootsteps(plan, f_pos_current, T_current, current_foot)

    % Initialize relevant variables from class ones
    num_steps = plan.num_steps;
    dT = plan.dT;
    t_imp = plan.t_imp_des;
    t_imp(current_foot) = T_current;
    constraints = plan.constraints;
    m = plan.m;

    % Boolena check to see if we are at the first iteration of a new plan
    new_plan = true;
    
    % Optimization to place the other feet
    for i=(plan.current_foot):num_steps    
        % Current last foot position
        if new_plan
            % If it's the first step of a new plan, take the last position
            % from solv
            current = f_pos_current;
        else
            current = plan.f_pos_des(:,i);
        end


        % Candidates for update of feet position
        if i == 1
            candidates = plan.uni_store;    
        else
            candidates = plan.uni_store(:, round(t_imp(i)/plan.dT):end);
        end


        % Part of the candidates that satisfy the constraints 
        [possible, times] = plan.checkConstraints(current, candidates);

        % Best among possible evaluated by the cost function
        [best, impact_time] = plan.chooseBest(current, possible, times);
        t_imp(i+1) = t_imp(i) + impact_time;

        % Update of foot position vector and save it in the class
        % variable. This also updates the next foot position (L/R)
        plan.f_pos_des(:,i+1) = plan.placeFoot(best);
        
        % After the first step, it's not a new plan anymore 
        new_plan = false;

    end
    % Save impact times in the class variables
    plan.t_imp_des = t_imp;
end