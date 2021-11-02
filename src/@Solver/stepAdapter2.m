function stepAdapter2(solv, iter)
% Function that solves the quadratic programming problem in the step
% adapter of Shafiee's paper. All quantities are reported with the same
% notation as the paper

% Modify reference frame and compute nominal quantities
ZMP_nom_rel = solv.f_pos_des(1:2, solv.f_iter) - solv.f_pos_des(1:2, solv.f_iter-1); 
gamma_nom_rel = solv.dcm_traj_des_eos_store(1:2, solv.f_iter) - solv.f_pos_des(1:2, solv.f_iter); 
T_nom_rel = solv.t_imp_des(solv.f_iter) - solv.t_curr;
sigma_nom_rel=exp(T_nom_rel*solv.omega); 

% ZMP exponential interpolation: currently commented away as it performs worse than the
% alternative
goal = ZMP_nom_rel;
start = [0;0];
% T is singleSUpport only. Thus we multiply by the inverse of solv.DSpar
sigma = exp((solv.t_imp_des(solv.f_iter+1)-solv.t_imp_des(solv.f_iter))*(1/solv.DSpar)*solv.omega);
A = ((goal-start)*sigma)/(1-sigma);
B = (start-(goal*sigma))/(1-sigma);
% Why is 0.1 here? Is it a way of stopping at the end of the Single Support
% phase?
ZMP_2 = A*exp(-(-0.1 + solv.t_imp_des(solv.f_iter) - solv.t_imp_des(solv.f_iter-1))*solv.omega) + B;

% ZMP from vrp
ZMP_1 = solv.vrp - solv.f_pos_des(1:2, solv.f_iter-1);
delta = ZMP_2-ZMP_1;


% Cost function
H = blkdiag(solv.h11, solv.h12, solv.h21, solv.h22, solv.h3);
f = [-solv.h11*ZMP_nom_rel(1); -solv.h12*ZMP_nom_rel(2); ...
    -solv.h21*gamma_nom_rel(1); -solv.h22*gamma_nom_rel(2);...
    -solv.h3*sigma_nom_rel]; % Note: it's a column

target = [ZMP_nom_rel(1); ZMP_nom_rel(2); gamma_nom_rel(1); gamma_nom_rel(2); sigma_nom_rel];

% Equality constraint
% relative_dcm = solv.dcm_traj_des_eos_store(:, solv.f_iter) - solv.f_pos_des(1:2, solv.f_iter-1);
relative_dcm = solv.dcm_pos - solv.f_pos_des(1:2, solv.f_iter-1);
coeff = ZMP_2 - relative_dcm - (delta/2);
Aeq = [eye(2), eye(2), coeff];
beq = ZMP_1 + (delta/2);



% Bounds. 
% Compute the center of the admissible rectangle according to which foot is
% being placed. 
% NOTE: implemented like this these make sense only in straight walking
% left to right! (0.25 should be changed)
if solv.left
    base = [0.25; -2*solv.m];
else
    base = [0.25; +2*solv.m];
end
% Actual numerical bounds on ZMP
zmp_max = base + [solv.ZMP_max_diff_x; solv.ZMP_max_diff_y];
zmp_min = base - [solv.ZMP_max_diff_x; solv.ZMP_max_diff_y];
% Bounds on sigma 
sigma_max = exp(1*solv.omega);
sigma_min = exp(0.1*solv.omega);

A = [eye(2),zeros(2),zeros(2,1); ...
       -eye(2),zeros(2),zeros(2,1); ...
       zeros(1,4),1; ...
       zeros(1,4),-1];
b = [zmp_max;-zmp_min;sigma_max;-sigma_min];


% Dismiss output message
lb = [];
ub = [];
x0=[];
options = optimset('Display', 'off');

% Solution computation. 1-2 is ZMPrel, 3-4 is gamma, 5 is sigma
solution = quadprog(H, f, A, b, Aeq, beq, lb, ub, x0, options);

solv.quadprog_sol_store(1:2,iter) = solution(1:2) + solv.f_pos_des(1:2, solv.f_iter-1); % foot position
solv.quadprog_sol_store(3:4,iter) = solution(3:4) + solv.quadprog_sol_store(1:2,iter);  % DCMeos
solv.quadprog_sol_store(5,iter) = log(solution(5))/solv.omega+0*solv.t_imp_des(solv.f_iter-1)+solv.t_curr; % Time

end


