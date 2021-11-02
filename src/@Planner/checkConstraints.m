        function [possible, times] = checkConstraints(plan, current, candidates)
            
            possible = []; 
            times = [];

            for i=1:length(candidates(1,:))
                
                % Compute constrained quantities
                delta_x = sqrt(((current(1)-candidates(1,i))^2) +  ((current(2)-candidates(2,i))^2));
                % Project this over x and y local directions of the robot
                phi = atan2(candidates(2,i) - current(2),candidates(1,i) - current(1)) + current(3);
                x_dist = delta_x * cos(phi);
                y_dist = delta_x * sin(phi);
                
                delta_t = i*plan.dT;
                delta_theta = abs(current(3) - candidates(3,i));

                % Check conditions
                condition_1 = (x_dist < 0.25);
                condition_2 = (abs(y_dist) > 0.15 && abs(y_dist) < 0.25);
                condition_3 = (delta_t>plan.t_min && delta_t<plan.t_max);
                condition_4 = (delta_theta <= plan.theta_max);

                % If feasible, add to possible solutions vector
                if (condition_1 && condition_2 && condition_3 && condition_4)
                    possible(:, end+1) = candidates(:,i);
                    times(end+1) = i*plan.dT;
                end
            end



        end