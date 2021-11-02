%% ________ Parameter initialization ___________

% All parameters are saved inside a struct that is passed to the class
% initializer
pars = struct;

%% TIMING
pars.T = 30; % Simulation time [s]
pars.dT = 0.01; % Discretization step [s]

% Number of steps to plan
pars.num_steps = 16;

%% Humanoid parameters
pars.m = 0.2; % Nominal distance between robot feet is 2*m [m]
pars.f_pos = zeros(3,pars.num_steps); % Feet position vector
pars.t_imp = zeros(1,pars.num_steps); % Impact times vector
pars.left = true; % Boolean that determines the next steps' foot
pars.h_com = 0.75;  % CoM height in [m]
pars.DSpar = 0.2; % Double support time parameter
pars.dcm_initial = [1;1-pars.m ];  % [x, y]  initial dcm position [m]
pars.dcm_traj_store=zeros(2,pars.T/pars.dT);
pars.dcm_vel_store=zeros(2,pars.T/pars.dT);
pars.dcm_traj_des_eos_store=zeros(2,pars.num_steps);
% Step adapter gains
% ZMP
pars.h11 = 100; 
pars.h12 = 100; 
% DCMeos
pars.h21 = 500;
pars.h22 = 500;
% Timing
pars.h3  = 0.01; 
% Step adapter bounds
pars.ZMP_max_diff_x = 0.15;
pars.ZMP_max_diff_y = 0.05;
pars.sigma_max_diff = 2;
pars.sigma_min_diff = 1;

%% Constants
pars.g = 9.81;  % gravitational acceleration constant [m/s^2]
pars.omega = sqrt(pars.g/pars.h_com); % LIP pulse [1/s]
pars.mass=39; % [kg]

%% Gains
pars.Kxi=1.1*eye(2);

%% ________________ Planner parameters _________________
plan_pars = struct;

%% Timing
plan_pars.T = 30; % Simulation time [s]
plan_pars.dT = 0.01; % Discretization step [s]

% Number of steps to plan
plan_pars.num_steps = pars.num_steps;

%% INITIAL TRAJECTORY
plan_pars.x_start = [1 ; 1]; % initial position [m]
plan_pars.x_goal = [9; 1]; % Goal position [m]
plan_pars.x_uni = [0.9; 1; 0];  % Unicycle initial position [x, y, theta]
plan_pars.d = [0.1; 0]; % Auxiliary point shift for controllability [m]
% storage of the trajectory of the auxiliary point
plan_pars.store_traj = zeros(3,plan_pars.T/plan_pars.dT);
% storage of the velocity of the auxiliary point
plan_pars.store_vel= zeros(3,plan_pars.T/plan_pars.dT);
% Unicycle controller gain
plan_pars.K_uni = [[0.1,0];[0,0.1]];

%% Feet related parameters
plan_pars.m = pars.m; % Nominal distance between robot feet is 2*m [m] (same as above)
plan_pars.f_pos = zeros(3,plan_pars.num_steps); % Feet position vector
plan_pars.t_imp = zeros(1,plan_pars.num_steps); % Impact times vector
plan_pars.left = pars.left; % Boolean that determines the next steps' foot
plan_pars.DSpar = 0.5; % Double support time parameter
plan_pars.omega = pars.omega; % LIP pulse [1/s]

% Step constraints
plan_pars.t_min = 0.1; % [s]
plan_pars.t_max = 1;   % [s]
plan_pars.d_max = 0.5;   % [m]
plan_pars.theta_max = 0.3;

% Dafarra optiimization gains
plan_pars.Kt = 0.015;
plan_pars.Kx = 1;
