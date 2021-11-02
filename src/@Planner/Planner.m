classdef Planner < dynamicprops
    properties(GetAccess=public)
        % Timing
        T % Total simulation time
        dT % Unicycle interpolation time step
        
        % Unicycle
        x_start
        x_goal
        uni_pos % position
        uni_store % Storage
        uni_vel_store % Storage of velocity
        d % controllability displacement
        K_uni % Controller gain
        
        % Footsteps
        num_steps % Total number of desired steps
        current_foot % Current step
        m % Distance between legs
        f_pos_des % desired footstep position
        t_imp_des % Desired impact times
        left % Boolean to determine current step foot
        DSpar % Fraction of time spent in double support phase
        
        % DCMeos and ZMP storage
        zmp_traj_store
        dcm_traj_des_eos_store
        omega % LIP pulse used in getDCMeosAndZMP
        
        % Constraints
        t_min
        t_max
        theta_max
        d_max
        constraints % Vector containing all step constraints
        
        % Dafarra optimization gains
        Kt
        Kx

    end
    
    
    methods (Access = public) 
        function plan = Planner(pars)
            
            plan.T = pars.T;
            plan.dT = pars.dT;
            
            %% Unicycle trajectory
            % Unicycle initial position
            plan.x_start = pars.x_start;
            % Unicycle final position
            plan.x_goal = pars.x_goal;
            % Unicycle trajectory storage
            plan.uni_store = pars.store_traj;
            % Unicycle velocity storage
            plan.uni_vel_store = pars.store_vel;
            % Unicycle initial position
            plan.uni_pos = pars.x_uni;  % x, y, theta
            % Auxiliary point shift for controllability [m]
            plan.d = pars.d;
            % Unicycle Controller gain
            plan.K_uni = pars.K_uni;            
            
            %% Feet
            plan.num_steps = pars.num_steps; % Number of steps to plan and simulate
            plan.m = pars.m; % Nominal distance between feet is 2*m [m]
            plan.f_pos_des = pars.f_pos; % Feet position vector
            plan.t_imp_des = pars.t_imp; % Impact times vector
            plan.left = pars.left; % Boolean that determines the next steps' foot
            plan.DSpar = pars.DSpar; % Double support time parameter
            % initial feet position [m] and impact time [s]
            plan.f_pos_des(:,1) = plan.placeFoot(plan.uni_pos(:));
            plan.t_imp_des(1) = 0;
            plan.omega = pars.omega; % LIP pulse used in getDCMeosAndZMP

            % Feet Constraints
            plan.t_min = pars.t_min;
            plan.t_max = pars.t_max;
            plan.d_max = pars.d_max;
            plan.theta_max = pars.theta_max;
            plan.constraints = [plan.t_min, plan.t_max, plan.d_max, plan.theta_max];
            
            % Dafarra optimization parameters
            plan.Kt = pars.Kt;
            plan.Kx = pars.Kx;

             
        end
        
        planNominalTrajectory(plan, x_current, t_current, left_current, current_foot)
        
        [new_state] = unicycleKin(plan, x_uni, u);
        
        [possible, times] = checkConstraints(plan, current, candidates);
        
        [best, impact] = chooseBest(plan, current, possible, times);
        
        sampleUnicycle(plan, x_start, x_goal, T_current);

        genNominalFootsteps(plan, f_pos_current, T_current, current_foot);

        [feetPos] = placeFoot(plan, x);        
        
        getDCMeosAndZMP(plan);
        
        matrix = rotateAroundZ(~, angle)
    end
    
end