function gates = generateGates()
%GENERATEGATES generates a vector of gates.

    N_gates = 3;

    %% Adjoint gates:
    for g = 1:N_gates % + 1 for start gate
        % Id
        gates(g).order = g;

        % Gate size
        gates(g).size         = [0.1 0.1]; % Height, Width

        % Tolerances
        gates(g).time_tol         = 10;
        gates(g).position_tol     = 0.1;
        gates(g).velocity_tol     = 10;
        gates(g).orientation_tol  = pi/2; % NOT SURE psi is pi to -pi
        gates(g).spin_tol         = 50*(2*pi/360); % angular velocity
    end

    u_hover = [13.7, 0, 0, 0];

    %% GATE 1
    gates(1).position    = zeros(1, 3);
    gates(1).velocity    = zeros(1, 3);
    gates(1).orientation = zeros(1, 3);
    gates(1).spin        = zeros(1, 3);

    gates(1).guess_time = 0;
    gates(1).time_min   = 0;
    gates(1).time_max   = 0;

    gates(1).guess_state = [gates(1).position, ...
                            gates(1).velocity, ...
                            gates(1).orientation, ...
                            gates(1).spin];
    % Fixed initial state
    gates(1).state_min   = gates(1).guess_state;
    gates(1).state_max   = gates(1).guess_state;

    gates(1).guess_control = u_hover;

    %% GATE 2
    gates(2).position    = [0, 0, 1];
    gates(2).velocity    = zeros(1, 3);
    gates(2).orientation = zeros(1, 3);
    gates(2).spin        = zeros(1, 3);

    gates(2).guess_time = 1;
    gates(2).time_min   = 0;
    gates(2).time_max   = gates(2).time_tol;

    gates(2).guess_state = [gates(2).position, ...
                            gates(2).velocity, ...
                            gates(2).orientation, ...
                            gates(2).spin];
    gates(2).state_min = [gates(2).position    - gates(2).position_tol, ...
                          gates(2).velocity    - gates(2).velocity_tol, ...
                          gates(2).orientation - gates(2).orientation_tol, ...
                          gates(2).spin        - gates(2).spin_tol];
    gates(2).state_max = [gates(2).position    + gates(2).position_tol, ...
                          gates(2).velocity    + gates(2).velocity_tol, ...
                          gates(2).orientation + gates(2).orientation_tol, ...
                          gates(2).spin        + gates(2).spin_tol];

    gates(2).guess_control = u_hover;

    %% GATE 3
    gates(3).position    = [0, 0, 2];
    gates(3).velocity    = zeros(1, 3);
    gates(3).orientation = zeros(1, 3);
    gates(3).spin        = zeros(1, 3);

    gates(3).guess_time = 2;
    gates(3).time_min   = 0;
    gates(3).time_max   = gates(3).time_tol;

    gates(3).guess_state = [gates(3).position, ...
                            gates(3).velocity, ...
                            gates(3).orientation, ...
                            gates(3).spin];
    gates(3).state_min = [gates(3).position    - gates(3).position_tol, ...
                          gates(3).velocity    - gates(3).velocity_tol, ...
                          gates(3).orientation - gates(3).orientation_tol, ...
                          gates(3).spin        - gates(3).spin_tol];
    gates(3).state_max = [gates(3).position    + gates(3).position_tol, ...
                          gates(3).velocity    + gates(3).velocity_tol, ...
                          gates(3).orientation + gates(3).orientation_tol, ...
                          gates(3).spin        + gates(3).spin_tol];

    gates(3).guess_control = u_hover;

    % Sanity check that there are as many gates as expected.
    assert(length(gates) == N_gates);
end


