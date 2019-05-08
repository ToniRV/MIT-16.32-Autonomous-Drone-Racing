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


u_hover = [13.7, 0, 0, 0];

t0                              = 0;
%t1                              = 0.2;
%t2                              = 0.8;
tf                              = Quad.sim_time; % No clue, it is free!
t_tol                           = 10;

% Time bounds at each phase:
% Each new phase starts with the previous phase time.
t_min = [t0, tf - t_tol];
t_max = [t0, tf + t_tol];

gate_1.order = 1;

gate_1.position = [0, 0, 1];
gate_1.orientation = [0, 0, 0];
gate_1.size = [10 10]; % Height, Width

% Tolerances
gate_1.time_tol = 10;
gate_1.position_tol = 0.1;
gate_1.velocity_tol = 0.1;
gate_1.orientation_tol = 0.1;
gate_1.spin_tol = 0.1; % angular velocity

gate_1.guess_time = 1;
gate_1.time_min = 0;
gate_1.time_max = 10;

gate_1.guess_state = [gate_1.position, zeros(1, 3),...
                      gate_1.orientation, zeros(1, 3)];
gate_1.state_min = [gate_1.position - gate_1.position_tol, ...
                    zeros(1, 3), ...
                    gate_1.orientation - gate_1.orientation_tol, ...
                    zeros(1, 3)];
gate_1.state_max = [gate_1.position + gate_1.position_tol, ...
                    zeros(1, 3), ...
                    gate_1.orientation + gate_1.orientation_tol, ...
                    zeros(1, 3)];
                  
gate_1.guess_control = u_hover;

gate_0.order = 0;

gate_0.position = [0, 0, 0];
gate_0.orientation = [0, 0, 0];
gate_0.size = [10 10]; % Height, Width

% Tolerances
gate_0.time_tol = 10;
gate_0.position_tol = 0.1;
gate_0.velocity_tol = 0.1;
gate_0.orientation_tol = 0.1;
gate_0.spin_tol = 0.1; % angular velocity

gate_0.guess_time = 0;
gate_0.time_min = 0;
gate_0.time_max = 0;

gate_0.guess_state = zeros(1, N_states);
gate_0.state_min = gate_0.state;
gate_0.state_max = gate_0.state;

gate_0.guess_control = u_hover;

gates = [gate_0; gate_1];

                  %x %y
x_endpoint_min = [gate_0.state_min;
                  gate_1.state_min];

x_endpoint_max = [gate_0.state_max;
                  gate_1.state_max];

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
    bounds.phase(p).initialstate.lower = x_endpoint_min(p, :);
    bounds.phase(p).initialstate.upper = x_endpoint_max(p, :);

    % Fixed final state for all phases
    bounds.phase(p).finalstate.lower   = x_endpoint_min(p + 1, :);
    bounds.phase(p).finalstate.upper   = x_endpoint_max(p + 1, :);

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
x_0  = x_endpoint_min(1,:);
x_tf = x_endpoint_min(2,:);


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


% % PHASE 2
% p = 2;
% guess.phase(p).time    = [0.2; 0.8];
% guess.phase(p).state   = [0.05 0.45; 0.37 0.52];
% guess.phase(p).control = [0.5; 0.5];
% guess.phase(p).integral = 1;
%
% % PHASE 3
% p = 3;
% guess.phase(p).time    = [t0; tf];
% guess.phase(p).state   = [0.37 0.52; x_tf];
% guess.phase(p).control = [u_max; u_max];
% guess.phase(p).integral = 1;

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

%%
%-------------------------------------------------------------------------%
%---------------------- Plot Solution ------------------------------------%
%-------------------------------------------------------------------------%

% Plot State History
f = figure('DefaultAxesFontSize', 16);
f.Name = 'MultiPhase';
subplot(4,3,1)
title('X Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,1), 'LineWidth', 2)
xlabel('t'), ylabel('x')

subplot(4,3,2)
title('Y Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,2), 'LineWidth', 2)
xlabel('t'), ylabel('y')

subplot(4,3,3)
title('Z Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,3), 'LineWidth', 2)
xlabel('t'), ylabel('z')

subplot(4,3,4)
title('Velocity X Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,4), 'LineWidth', 2)
xlabel('t'), ylabel('Vel X')

subplot(4,3,5)
title('Velocity Y Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,5), 'LineWidth', 2)
xlabel('t'), ylabel('Vel Y')

subplot(4,3,6)
title('Velocity Z Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,6), 'LineWidth', 2)
xlabel('t'), ylabel('Vel Z')

subplot(4,3,7)
title('Phi Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,7), 'LineWidth', 2)
xlabel('t'), ylabel('Phi')

subplot(4,3,8)
title('Theta Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,8), 'LineWidth', 2)
xlabel('t'), ylabel('Theta')

subplot(4,3,9)
title('Psi Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,9), 'LineWidth', 2)
xlabel('t'), ylabel('Psi')

subplot(4,3,10)
title('p Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,10), 'LineWidth', 2)
xlabel('t'), ylabel('p')

subplot(4,3,11)
title('q Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,11), 'LineWidth', 2)
xlabel('t'), ylabel('q')

subplot(4,3,12)
title('r Numerical')
hold on
plot(solution.phase(1).time, solution.phase(1).state(:,12), 'LineWidth', 2)
xlabel('t'), ylabel('r')

% subplot(3,2,3:4)
% title('State path (x vs. v)')
% hold on
% plot(solution.phase(1).state(:,1), solution.phase(1).state(:,2), 'LineWidth', 2)
% plot(solution.phase(2).state(:,1), solution.phase(2).state(:,2), 'LineWidth', 2)
% plot(solution.phase(3).state(:,1), solution.phase(3).state(:,2), 'LineWidth', 2)
% xlabel('x'), ylabel('v')
%
% subplot(3,2,5:6)
% title('U Numerical')
% hold on
% plot(solution.phase(1).time, solution.phase(1).control, 'LineWidth', 2)
% plot(solution.phase(2).time, solution.phase(2).control, 'LineWidth', 2)
% plot(solution.phase(3).time, solution.phase(3).control, 'LineWidth', 2)
% xlabel('t'), ylabel('u')
