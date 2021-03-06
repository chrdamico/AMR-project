classdef Solver < dynamicprops
    properties(GetAccess=public)

        % Footsteps
        m % Distance between legs
        f_pos_des % desired footstep position
        t_imp_des % Desired impact times
        f_iter  % Current foot iteration
        
        % vrp
        vrp_des % Nominal vrp from controller
        vrp     % Actual vrp including push
        
        % Dcm        
        dcm_pos
        dcm_vel
        dcm_pos_des
        dcm_pos_des_eos
        dcm_pos_des_eos_prev % prev=previous
        dcm_vel_des
        
        % zmp
        zmp_pos_des
        zmp_pos_des_prev
        single_support % :True/False therefore double_support = not single_support

        % if single support is true, this indicates if we are using left foot (true)
        % or right foot (false). 
        left

        % Timing
        t_curr % Current time
        tstep % duration of the current step
        tstep_prev
        DSpar % Double support time parameter
        tstep_i % initial time of the current step
        tstep_e % end time of the current step (tstep_i+tstep)
        T % simulation time
        dT % discretization step

        % General dynamical constants
        omega
        num_steps
        mass

        % Simplified model control gain
        Kxi

        % Step Adapter parameters
        h11
        h12
        h21
        h22
        h3

        % Step Adapter bounds
        ZMP_max_diff_x
        ZMP_max_diff_y
        sigma_max_diff
        sigma_min_diff
        % storage
        dcm_traj_store
        dcm_traj_des_store
        dcm_traj_des_eos_store
        dcm_vel_des_store
        quadprog_sol_store
        vrp_store
        vrp_des_store
        
        % Require replanning boolean (re-plan the trajectory only if
        % footsteps are changed by the step adapter
        replan
        
        temp

    end

    methods

        function solv = Solver(pars)
            % Footstep counter
            solv.f_iter=2;
            % Time
            solv.T = pars.T; % Total time
            solv.dT = pars.dT; % Discretization step
            solv.t_curr=0; % Initialization of the current time

            %% Humanoid
            solv.num_steps = pars.num_steps; % Number of steps to plan and simulate
            solv.m = pars.m; % Nominal distance between feet is 2*m [m]
            solv.f_pos_des = pars.f_pos; % Feet position vector
            solv.t_imp_des = pars.t_imp; % Impact times vector
            solv.left = pars.left; % Boolean that determines the next steps' foot
            solv.DSpar = pars.DSpar; % Double support time parameter
            solv.dcm_traj_des_store=pars.dcm_traj_store; % dmc position storage [m]
            solv.dcm_vel_des_store=pars.dcm_vel_store; % dcm velocity storage[m/s]
            solv.dcm_traj_des_eos_store=pars.dcm_traj_des_eos_store; % dcm pos at end of step [m]

            % Initial vrp position
            solv.vrp = solv.f_pos_des(1:2,2);

            % Constraints on quadprog optimization
            solv.ZMP_max_diff_x = pars.ZMP_max_diff_x;
            solv.ZMP_max_diff_y = pars.ZMP_max_diff_y;
            solv.sigma_max_diff = pars.sigma_max_diff;
            solv.sigma_min_diff = pars.sigma_min_diff;

            % Quadprog storage
            solv.quadprog_sol_store = zeros(5, round(solv.T/solv.dT));

            %% Dynamical onstants
            solv.omega=pars.omega;
            solv.mass=pars.mass;
            %% Gains
            % Step adapter solver
            solv.h11 = pars.h11;
            solv.h12 = pars.h12;
            solv.h21 = pars.h21;
            solv.h22 = pars.h22;
            solv.h3 = pars.h3;
            % Simplified model controller
            solv.Kxi=pars.Kxi;




        end

        function cycle(solv,iter)
            
            %% Step adapter
            % Compute the step adapter solution from iteration 2 only in
            % single support phases (starting after iteration 1000)
            if iter>50 
                if solv.single_support 
                    solv.stepAdapter(iter)
                    % Accept new goals only if the change is big enough
                    zmp_err=abs(solv.f_pos_des(1:2,solv.f_iter)-solv.quadprog_sol_store(1:2,iter));
                    dcm_eos_err=abs(solv.dcm_pos_des_eos-solv.quadprog_sol_store(3:4,iter));
                    time_err=abs(solv.t_imp_des(solv.f_iter)-solv.quadprog_sol_store(5,iter));  
                    % New zmp position
                    if norm(zmp_err)>1e-2
                        solv.f_pos_des(1:2,solv.f_iter)=solv.quadprog_sol_store(1:2,iter);
                        solv.replan = true;
                    end
                    % New DCMeos position
                    if norm(dcm_eos_err)>1e-2
                        solv.dcm_traj_des_eos_store(:, solv.f_iter)=solv.quadprog_sol_store(3:4,iter);
                        solv.replan = true;
                    end
                    % New timing
                    if (time_err > 0.01) 
                        if isreal(solv.quadprog_sol_store(5,iter))
                            solv.t_imp_des(solv.f_iter) = solv.quadprog_sol_store(5,iter);
                        else
                            warning('ignored complex time solution')
                        end
                        solv.replan = true;
                    end

                else
                    solv.quadprog_sol_store(:,iter) = solv.quadprog_sol_store(:,iter-1);
                end
                
            end
            
          

            % Initializing variables used in smoothing and stepAdapter
            solv.zmp_pos_des = solv.f_pos_des(1:2,solv.f_iter-1); % Misleading name! It's the currrently last completed step!
            solv.dcm_pos_des_eos = solv.dcm_traj_des_eos_store(:, solv.f_iter);
            solv.tstep = solv.t_imp_des(solv.f_iter);
            
            %% Smoothing

            % Compute the exponentially smoothed trajectory of the dcm
            solv.support(); % Detect if single or double support and instantiate needed variables
            dcm_des=solv.smoothing(); % Trajectory smoothing
            
            % Store results
            solv.dcm_pos_des=dcm_des(1:2);
            solv.dcm_vel_des=dcm_des(3:4);

            solv.dcm_traj_des_store(:,iter)=dcm_des(1:2);
            solv.dcm_vel_des_store(:,iter)=dcm_des(3:4);

            % Compute the control using an LIP model (from Shafiee)
            solv.vrp_des = simplifiedModelControl(solv, iter);

            % Push every once in a while. Otherwise, just apply the control
            % be applied

            solv.vrp = solv.vrp_des;

            
            % Store actual vrp value
            solv.vrp_store(:,iter) = solv.vrp;
            solv.vrp_des_store(:,iter) = solv.vrp_des;
            
            % Update dcm_pos
            if solv.f_iter == 9 && solv.t_curr - solv.t_imp_des(solv.f_iter-1) >= 0.15 && solv.t_curr - solv.t_imp_des(solv.f_iter-1) < 0.25
            push(solv,iter);
            else
            LIPdynamics(solv);
            end

            solv.dcm_traj_store(1:2,iter) = solv.dcm_pos;
        end

        push(solv,iter);

        LIPdynamics(solv);
   
       
        vrp = simplifiedModelControl(solv, iter);
    

        support(solv);

        dcm=smoothing(solv);
        
        a=getCoeff(solv);
        
        stepAdapter(solv, iter);

        stepAdapter2(solv, iter)
        
        stepAdapterModified(solv, iter);
        
    end
end
