        function [new_state] = unicycleKin(plan, x_uni, u)
            % Kinematic function of a unicycle. Computes a new state from velocity
            % commands

            x = x_uni(1);
            y = x_uni(2);
            theta = x_uni(3);

            v = u(1);
            w = u(2);

            x = x + plan.dT*v*cos(theta);
            y = y + plan.dT*v*sin(theta);
            theta = theta + w*plan.dT;

            new_state = [x;y;theta];

        end