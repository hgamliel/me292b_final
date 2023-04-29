% run genetic algorithm

n_torques = 500;                        % number of input torque time steps
nvars = 1 + n_torques*2;                % number of design variables

param_animate = false;                  % do not animate

% MATLAB's global optimization toolbox
A = []; b = []; Aeq = []; beq = []; nonlcon = [];
tau_max = 500;                              % actuator torque limit [N*m]
lb = [0, zeros(1,n_torques*2)];             % lower bounds
ub = [5, tau_max*ones(1,n_torques*2)];      % upper bounds

options = optimoptions('ga', 'Display', 'iter', 'PlotFcn', {@gaplotbestf}, ...
    'OutputFcn', @save_intermediate_states);

% issues: T_release is one of too many variables, not being well optimized
% for
% initial population, one row all zeros
% less crossover
% maybe make the u array a polynomial function instead? sine fit function?

[x_optim, fval] = ...
    ga(@simulate_brachiation, nvars, A, b, Aeq, beq, lb, ub, nonlcon, options);


% save states at intermediate generations
function [state, options, optchanged] = save_intermediate_states(options, state, flag)
    generation = state.Generation;
    optchanged = false;

    if mod(generation, 100) == 0
        [~, ind] = min(state.Score);
        x = state.Population(ind,:);
        save(['x_' num2str(generation)]);
    end
end

%% Attempt to solve pi_b using GA. Plot best cost and mean cost.

% f = @(x) pi_bf(x);
% [PI, Orig, Lambda] = geneticAlgorithm(12, 10^-6, 100, 50, 1, f);
% 
% for i = 1:size(PI,1)
%     b_cost(i) = PI(i,1);
%     p_cost(i) = mean(PI(i,1:12));
% end
% 
% figure
% plot(p_cost);
% hold on
% plot(b_cost);
% legend('Parent Cost','Best Cost');
% title('Average Parent Cost vs. Best Cost');
% xlabel('number of generations');
% ylabel('cost');
% print('Tight Tolerance','-dpng');
% 
% clear;
