% run genetic algorithm

n = 10;                         % number of sum of sines terms
nvars = 1 + n*6;                % number of design variables

param_animate = false;          % do not animate

% MATLAB's global optimization toolbox
A = []; b = []; Aeq = []; beq = []; nonlcon = [];
lb = [0.01, -inf*ones(1,n*6)];  % lower bounds
ub = [3, inf*ones(1,n*6)];      % upper bounds

initPop = [0.5 1500/n*ones(1,n) zeros(1,n) pi/2*ones(1,n) ...
    0/n*ones(1,n) zeros(1,n) pi/2*ones(1,n)];
% worse initial guess, demonstrate how GA improves
initPop = [0.5 900/n*ones(1,n) zeros(1,n) pi/2*ones(1,n) ...
    100/n*ones(1,n) zeros(1,n) pi/2*ones(1,n)];

options = optimoptions('ga', 'Display', 'iter', 'PlotFcn', {@gaplotbestf}, ...
    'OutputFcn', @save_intermediate_states, 'CrossoverFraction', 0.5, ...
    'PopulationSize', 1000, 'InitialPopulationMatrix', initPop, ...
    'MaxGenerations', 100);

[x_optim, fval] = ...
    ga(@simulate_brachiation, nvars, A, b, Aeq, beq, lb, ub, nonlcon, options);


% save states at intermediate generations
function [state, options, optchanged] = save_intermediate_states(options, state, flag)
    generation = state.Generation;
    optchanged = false;

    if mod(generation, 25) == 0
        [~, ind] = min(state.Score);
        x = state.Population(ind,:);
        save(['energy_x_gen' num2str(generation)]);
    end
end
