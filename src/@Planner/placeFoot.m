        function [feetPos] = placeFoot(plan, x)
            % Initialize relevant variables from class ones
            m = plan.m;
            left = plan.left;

            % Check if it's a right or left foot step and update for the
            % next one
            if left
                displacement = [0; m];
                plan.left = false;
            else
                displacement = [0; -m];
                plan.left = true;
            end

            % Compute foot position
            R = plan.rotateAroundZ(x(3));
            R = R(1:2,1:2);

            position = x(1:2) + R*displacement;
            feetPos = [position; x(3)];

        end