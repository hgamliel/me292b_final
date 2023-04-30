% run genetic algorithm

n = 10;                         % number of sum of sines terms
nvars = 1 + n*6;                % number of design variables

param_animate = false;          % do not animate

% MATLAB's global optimization toolbox
A = []; b = []; Aeq = []; beq = []; nonlcon = [];
lb = [0, -inf*ones(1,n*6)];     % lower bounds
ub = [5, inf*ones(1,n*6)];      % upper bounds

initPop = [1.7 -1000/n*ones(1,n) zeros(1,n) pi/2*ones(1,n) ...
    -150/n*ones(1,n) zeros(1,n) pi/2*ones(1,n)];

options = optimoptions('ga', 'Display', 'iter', 'PlotFcn', {@gaplotbestf}, ...
    'OutputFcn', @save_intermediate_states, 'CrossoverFraction', 0.5, ...
    'PopulationSize', 1000, 'InitialPopulationMatrix', initPop);

[x_optim, fval] = ...
    ga(@simulate_brachiation, nvars, A, b, Aeq, beq, lb, ub, nonlcon, options);


% save states at intermediate generations
function [state, options, optchanged] = save_intermediate_states(options, state, flag)
    generation = state.Generation;
    optchanged = false;

    if mod(generation, 50) == 0
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
