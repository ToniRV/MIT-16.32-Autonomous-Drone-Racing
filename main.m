%------------------------- Drone Race Problem ----------------------------%
%-------------------------------------------------------------------------%
clear all;
close all; 
clc;

% Addpaths
addpath src
addpath src/utilities/
addpath src/utilities/java

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
%% Parse gates
[gates_data, max_pos, min_pos] = parseGatePositions();

state_tol = 2; % meters
max_pos = max_pos + state_tol;
min_pos = min_pos - state_tol;

%% 
gates = generateGates(); % Currently does not use parsed gates...

%%
N_phases = length(gates) - 1;
N_states = length(fieldnames(Quad.State));

% Create auxdata
auxdata.gates = gates;
auxdata.N_phases = N_phases;
                                    
x_min = [Quad.X_min Quad.Y_min Quad.Z_min ...
         Quad.X_dot_min Quad.Y_dot_min Quad.Z_dot_min ...
         Quad.phi_min Quad.theta_min Quad.psi_min ...
         Quad.p_min Quad.q_min Quad.r_min ];

x_max = [Quad.X_max Quad.Y_max Quad.Z_max ...
         Quad.X_dot_max Quad.Y_dot_max Quad.Z_dot_max ...
         Quad.phi_max Quad.theta_max Quad.psi_max ...
         Quad.p_max Quad.q_max Quad.r_max ];

u_min = [Quad.U1_min Quad.U2_min Quad.U3_min Quad.U4_min];
u_max = [Quad.U1_max Quad.U2_max Quad.U3_max Quad.U4_max];

for p = 1:N_phases
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

    %% Eventgroup constraints
    if p < N_phases
        % We have a +1 to account for time, and the last component is to
        % force the drone to traverse the gate
        bounds.eventgroup(p).lower = [zeros(1, N_states + 1), gates(p).vel_normal_tol]; 
        bounds.eventgroup(p).upper = [zeros(1, N_states + 1), 1.0];
    end
end

%%
%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%

for p = 1:N_phases
    guess.phase(p).time    = [gates(p).guess_time; gates(p + 1).guess_time];
    guess.phase(p).state   = [gates(p).guess_state; gates(p + 1).guess_state];
    guess.phase(p).control = [gates(p).guess_control; gates(p + 1).guess_control];
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
setup.auxdata                        = auxdata;
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
plotStates(N_phases, solution, Quad);

%% Plot Control History
plotControls(N_phases, solution, Quad);

%% Initialize the plot
hold off;
axis_limits = [Quad.X_min Quad.X_max ...
               Quad.Y_min Quad.Y_max ...
               Quad.Z_min Quad.Z_max];
initPlot(axis_limits);
plotQuadModel;
plotGates(gates);
plotVelocityCones(gates);

% Plot velocity vector
h = quiver3(0,0,0,0,0,0);

% Setup video recording
axis tight manual 
record_video = 0;
if record_video == 1
    set(gca,'nextplot','replacechildren'); 
    v = VideoWriter('peaks.avi');
    open(v);
end

% Accumulate states.
acc_state_x = [0];
acc_state_y = [0];
acc_state_z = [0];
acc_time = [0];
for p = 1:N_phases
    for idx = 1:size(solution.phase(p).state, 1)
        % Convert solution state to list of Quad.State
        Quad.State = vectorToState(solution.phase(p).state(idx, :));

        % Convert solution control to list of Quad.Control
        Quad.Control = vectorToControl(solution.phase(p).control(idx, :));
        
        % Plot Quad
        plotQuad
        
        % Plot velocity
        set(h, 'Xdata', Quad.State.X, 'Ydata', Quad.State.Y, 'Zdata', Quad.State.Z,...
               'Udata', Quad.State.X_dot, 'Vdata', Quad.State.Y_dot, 'Wdata', Quad.State.Z_dot);
        %plot3(Quad.State.X,Quad.State.Y,Quad.State.Z,'*r');
        % Plot trajectory
        drawnow
        
        acc_state_x = [acc_state_x, Quad.State.X];
        acc_state_y = [acc_state_y, Quad.State.Y];
        acc_state_z = [acc_state_z, Quad.State.Z];
        acc_time = [acc_time, solution.phase(p).time(idx)];
        
        % Record video.
        if record_video == 1
            frame = getframe(gcf);
            writeVideo(v,frame);
        end
    end
end


scatter3(acc_state_x,acc_state_y,acc_state_z,10,acc_time, 'LineWidth', 5)
colorbar

hold off;

if record_video == 1
    close(v);
end

%%


