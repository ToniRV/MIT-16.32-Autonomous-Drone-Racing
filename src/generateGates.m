function gates = generateGates()
%GENERATEGATES generates a vector of gates.

    N_gates = 5;

    %% Adjoint gates:
    for g = 1:N_gates % + 1 for start gate
        % Id
        gates(g).order = g;

        % Gate size
        gates(g).radius           = 0.7; % Height, Width

        % State guesses
        gates(4).orientation = zeros(1, 3);
        gates(4).spin        = zeros(1, 3);

        % Tolerances
        gates(g).time_tol         = 10;
        gates(g).position_tol     = 0.1;
        gates(g).velocity_tol     = 10;
        % This must be actually set smartly!
        gates(g).orientation_tol  = pi/8; % NOT SURE psi is pi to -pi
        gates(g).spin_tol         = 50*(2*pi/360); % angular velocity
        gates(g).vel_normal_tol   = 0.7; % Bound for dot prod of velocity direction to gate's normal

        gates(g).time_min   = 0;
        gates(g).time_max   = gates(g).time_tol;

        gates(g).guess_state = [gates(g).position, ...
                                gates(g).velocity, ...
                                gates(g).orientation, ...
                                gates(g).spin];
        gates(g).state_min = [gates(g).position    - gates(g).position_tol, ...
                              gates(g).velocity    - gates(g).velocity_tol, ...
                              gates(g).orientation - gates(g).orientation_tol, ...
                              gates(g).spin        - gates(g).spin_tol];
        gates(g).state_max = [gates(g).position    + gates(g).position_tol, ...
                              gates(g).velocity    + gates(g).velocity_tol, ...
                              gates(g).orientation + gates(g).orientation_tol, ...
                              gates(g).spin        + gates(g).spin_tol];
    end

    u_hover = [13.7, 0, 0, 0];

    %% GATE 1: start gate
    % This one is particular as we set the drone to start with 0
    % velocity.
    % Drone states:
    gates(1).normal      = [0, 0, 1];
    gates(1).position    = zeros(1, 3);
    gates(1).velocity    = zeros(1, 3);

    gates(1).guess_time = 0;

    gates(1).guess_control = u_hover;

    % Fixed initial time
    gates(1).time_min   = 0;
    gates(1).time_max   = 0;

    % Fixed initial state
    gates(1).state_min   = gates(1).guess_state;
    gates(1).state_max   = gates(1).guess_state;


    %% GATE 2
    gates(2).normal      = [0, 0, 1];
    gates(2).position    = [0, 0, 1];
    gates(2).velocity    = gates(2).normal;

    gates(2).guess_time = 1;

    gates(2).guess_control = u_hover;

    %% GATE 3
    gates(3).normal      = [0, 1, 0];
    gates(3).position    = [0, 2, 2];
    gates(3).velocity    = gates(3).normal;

    gates(3).guess_time = 2;

    gates(3).guess_control = u_hover;

    %% GATE 4
    gates(4).normal      = [0, 0, -1];
    gates(4).position    = [1, 3, 1];
    gates(4).velocity    = gates(4).normal;

    gates(4).guess_time = 3;

    gates(4).guess_control = u_hover;

    %% GATE 5
    gates(5).normal      = [1, 0, 0];
    gates(5).position    = [2, 1, 2];
    gates(5).velocity    = gates(5).normal;

    gates(5).guess_time = 4;

    gates(5).guess_control = u_hover;

    %% Sanity check that there are as many gates as expected.
    assert(length(gates) == N_gates);
end


