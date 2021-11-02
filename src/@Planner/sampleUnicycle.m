function sampleUnicycle(plan, x_start, x_goal, T_current)

    % Initialize relevant variables from class ones
    x_uni = x_start;
    x_start = x_uni(1:2);
    d = plan.d;
    T = plan.T;
    dT = plan.dT;
    K = plan.K_uni;

    % Compute desired position trajectories. The +1 at the end is added to
    % ensure that the dimension of the array is correct
    x_traj = linspace(x_start(1),x_goal(1),(T-T_current)/dT + 1);
    y_traj = linspace(x_start(2),x_goal(2),(T-T_current)/dT + 1);
    x_desired = [x_traj;y_traj];

    % Set the initial iteration for the unicycle motion
    if T_current == 0
        initial = 1;
    else
        initial = round(T_current/dT);
    end
    % Main loop
    for i = initial:round((T-T_current)/dT)  
        % Store result of the auxiliary point motion
        plan.uni_store(:,i) = [x_uni(1)+d(1); x_uni(2)+d(2); x_uni(3)];

        % Auxiliary control point
        R = plan.rotateAroundZ(x_uni(3));
        R = R(1:2,1:2);
        x_f = x_uni(1:2) + R*d;

        % Input from control law:
        B = [[cos(x_uni(3)), -d(1)*sin(x_uni(3))-d(2)*cos(x_uni(3))]; 
             [sin(x_uni(3)), +d(1)*cos(x_uni(3))-d(2)*sin(x_uni(3))]];

         
        u = inv(B)*((x_goal-x_start)/(T-T_current)-K*(x_f-x_desired(:,i-initial+1))); 

        plan.uni_vel_store(i)=u(1);

        % New state computation
        x_uni = plan.unicycleKin(x_uni, u);

    end

end