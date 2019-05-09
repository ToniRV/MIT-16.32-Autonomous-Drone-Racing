%------------------------- Drone Race Problem ----------------------------%
%-------------------------------------------------------------------------%
close all;
clc;

% Addpaths
addpath utilities

%-------------------------------------------------------------------------%
%------------- Provide and Set Up Quadcopter -----------------------------%
%-------------------------------------------------------------------------%

%% Initialize Variables
global Quad;

%% Initialize the plot
%initPlot;
%plotQuadModel;

% Init quad params
parametersQuad;

% Init state
initState;

%% Init control
% Quad.Control.U1 = 13.7;
% Quad.Control.U2 = 0;
% Quad.Control.U3 = 0;
% Quad.Control.U4 = 0;
%
% Quad.counter = Quad.counter + 1;
%
% while Quad.t_plot(Quad.counter - 1) < max(Quad.t_plot)
%     % Nonlinear Dynamics given inputs and current state.
%     nonlinearQuadrotorDynamics(Quad.State, Quad.Control);
%
%     % Update state.
%     updateState;
%
%     if(mod(Quad.counter, 3) == 0)
%         % Plot the Quadrotor's Position.
%         plotQuad
%         drawnow
%     end
%
%     % Next timestep.
%     Quad.counter = Quad.counter + 1;
% end

%% Setup and Solve Drone Race problem using GPOPS-II
%-------------------------------------------------------------------------%
%------------- Provide and Set Up All Bounds for Problem -----------------%
%-------------------------------------------------------------------------%
gpops_params = gpopsParams;
N_gates = gpops_params.N_gates;
N_states = length(fieldnames(Quad.State));

gates = generateGates(N_states, N_gates);
                                    %x %v
x_min = [Quad.X_min Quad.Y_min Quad.Z_min ...
         Quad.X_dot_min Quad.Y_dot_min Quad.Z_dot_min ...
         Quad.phi_min Quad.theta_min Quad.psi_min ...
         Quad.p_min Quad.q_min Quad.r_min ]; % TO FIX

x_max = [Quad.X_max Quad.Y_max Quad.Z_max ...
         Quad.X_dot_max Quad.Y_dot_max Quad.Z_dot_max ...
         Quad.phi_max Quad.theta_max Quad.psi_max ...
         Quad.p_max Quad.q_max Quad.r_max ]; % TO FIX

u_min = [Quad.U1_min Quad.U2_min Quad.U3_min Quad.U4_min];
u_max = [Quad.U1_max Quad.U2_max Quad.U3_max Quad.U4_max];

for p = 1:N_gates
    %% Time
    % Fixed initial time for all phases...
    bounds.phase(p).initialtime.lower  = gates(p).time_min;
    bounds.phase(p).initialtime.upper  = gates(p).time_max;

    % Free final time for all phases
    bounds.phase(p).finaltime.lower    = gates(p + 1).time_min;
    bounds.phase(p).finaltime.upper    = gates(p + 1).time_max;

    %% State
    % Fixed initial state for all phases
    bounds.phase(p).initialstate.lower = gates(p).state_min;
    bounds.phase(p).initialstate.upper = gates(p).state_max;

    % Fixed final state for all phases
    bounds.phase(p).finalstate.lower   = gates(p + 1).state_min;
    bounds.phase(p).finalstate.upper   = gates(p + 1).state_max;

    % Tolerance for state on each phase
    bounds.phase(p).state.lower        = x_min;
    bounds.phase(p).state.upper        = x_max;

    %% Control
    % Control bounds for each phase
    bounds.phase(p).control.lower      = u_min;
    bounds.phase(p).control.upper      = u_max;

    %% Integral
    % Integral bounds for each phase
    %bounds.phase(p).integral.lower     = integral_min(p);
    %bounds.phase(p).integral.upper     = integral_max(p); % NOT SURE

    %% Eventgroup constraints
    if p < N_gates
        bounds.eventgroup(p).lower = zeros(1, N_states);
        bounds.eventgroup(p).upper = zeros(1, N_states);
    end
end

%%
%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%

% straight_lines_guess = 1;
% if straight_lines_guess
%     for p = 1:N_gates
%         gate_to_gate_dist = gates(p + 1).position - gates(p).position;
%     end
% end

for p = 1:N_gates
    % PHASE 1
    guess.phase(p).time    = [gates(p).guess_time; gates(p + 1).guess_time];
    guess.phase(p).state   = [gates(p).guess_state; gates(p + 1).guess_state];
    guess.phase(p).control = [gates(p).guess_control; gates(p + 1).guess_control];
    %guess.phase(p).integral = 0;
end

%%
%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
%  mesh.method            = 'hp-PattersonRao';
%  mesh.tolerance         = 1e-10;
%  mesh.maxiterations     = 40;
%  mesh.colpointsmin      = 10;
%  mesh.colpointsmax      = 40;
% NumIntervals           = 4;
% mesh.phase.colpoints   = 6*ones(1,NumIntervals);
% mesh.phase.fraction    = ones(1,NumIntervals)/NumIntervals;

%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%
%-------------------------------------------------------------------------%
setup.name                           = 'Drone-Race';
setup.functions.continuous           = @continuous;
setup.functions.endpoint             = @endpoint;
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.auxdata                        = gpops_params;
setup.nlp.solver                     = 'ipopt';
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.derivatives.supplier           = 'sparseCD';
setup.derivatives.derivativelevel    = 'second';
setup.method                         = 'RPM-Differentiation';
%setup.mesh                           = mesh;

%%
%-------------------------------------------------------------------------%
%---------------------- Solve Problem Using GPOPS2 -----------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);
solution = output.result.solution;

%% Save Gpops solution
save('GPOPS_DroneRace_solution');

%% Reload Gpops
load('GPOPS_DroneRace_solution');

%%
%-------------------------------------------------------------------------%
%---------------------- Plot Solution ------------------------------------%
%-------------------------------------------------------------------------%

%% Plot State History
plotStates(N_gates, solution, Quad);

%% Plot Control History
plotControls(N_gates, solution);

%% Initialize the plot
hold off;
initPlot;

plotQuadModel;

plotGates(gates);

for p = 1:N_gates 
    for idx = 1:size(solution.phase(p).state, 1)
        % Convert solution state to list of Quad.State
        Quad.State = vectorToState(solution.phase(p).state(idx, :));

        % Convert solution control to list of Quad.Control
        Quad.Control = vectorToControl(solution.phase(p).control(idx, :));
        
        % Plot Quad
        plotQuad
        drawnow
    end
end
