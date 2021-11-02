        function [best, impact] = chooseBest(plan, current, possible, times)

            % Initial cost value (high so that any point is better)
            top = inf;
            % Optimization loop
            for i=1:length(possible(1,:))
                % Compute deltas and cost
                delta_x = sqrt(((current(1)-possible(1,i))^2) +  ((current(2)-possible(2,i))^2));
                delta_t = times(i);

                phi = plan.Kt/(delta_t^2) + plan.Kx*(delta_x^2);

                % Check if the new cost is better than current top
                if (phi<=top)
                    top = phi;
                    % Save the id of the best cost
                    best_id = i;
                end
            end

            % Return the best position for the foot's placement
            best = possible(:, best_id);
            impact = times(best_id);

        end