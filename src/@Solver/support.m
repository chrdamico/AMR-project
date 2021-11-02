function support(solv)
            current_foot = solv.f_iter-1;
            % Assume in double support
            solv.single_support=false;
            % DS time of the current step
            curr_ds=solv.DSpar*(solv.t_imp_des(current_foot+1)-solv.t_imp_des(current_foot));
            % In the first DSpar of the step and in the last DSpar
            % of the step we are in double support phase
            % In the first case, the refence step is the current one.
            % Whereas, for the latter, it's the next step.
            % All the other parameters follow the above philosophy.
            % If none of the cases apply we are in single support.
            if solv.t_curr<=solv.t_imp_des(current_foot)+curr_ds
                if current_foot>1
                    solv.tstep_prev=solv.t_imp_des(current_foot);
                    solv.tstep_i=solv.t_imp_des(current_foot)-solv.DSpar*(solv.t_imp_des(current_foot)-solv.t_imp_des(current_foot-1));%solv.tstep_prev;
                    solv.dcm_pos_des_eos_prev=solv.dcm_traj_des_eos_store(1:2,current_foot);
                    solv.zmp_pos_des_prev=solv.f_pos_des(1:2,current_foot-1);
                else       
                    solv.tstep_prev=solv.t_imp_des(current_foot);
                    solv.tstep_i=solv.t_imp_des(current_foot);
                    solv.dcm_pos_des_eos_prev=solv.dcm_traj_des_eos_store(1:2,current_foot);
                    solv.zmp_pos_des_prev=solv.f_pos_des(1:2,current_foot);
                end
                solv.tstep_e=solv.t_imp_des(current_foot)+curr_ds;
            elseif solv.t_curr>=solv.t_imp_des(current_foot+1)-curr_ds
                solv.tstep_i=solv.t_imp_des(current_foot+1)-curr_ds;
                solv.tstep_prev=solv.tstep;
                solv.dcm_pos_des_eos_prev=solv.dcm_traj_des_eos_store(1:2,current_foot+1);
                solv.zmp_pos_des_prev=solv.f_pos_des(1:2,current_foot);               
                if current_foot<solv.num_steps-1
                    tstep_succ=(solv.t_imp_des(current_foot+2)-solv.t_imp_des(current_foot+1));                    
                    solv.tstep=solv.t_imp_des(current_foot+2);
                    solv.dcm_pos_des_eos=solv.dcm_traj_des_eos_store(1:2,current_foot+2);
                    solv.zmp_pos_des=solv.f_pos_des(1:2,current_foot+1);
                else
                    tstep_succ=0;
                    solv.tstep=solv.t_imp_des(solv.num_steps);
                end
                solv.tstep_e=solv.t_imp_des(current_foot+1)+solv.DSpar*tstep_succ;
            else
                solv.single_support=true;
            end       
        end
        
        
