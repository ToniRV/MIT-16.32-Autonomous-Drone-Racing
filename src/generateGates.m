function gates = generateGates()
%GENERATEGATES generates a vector of gates.

    N_gates = 5;

    %% Adjoint gates:
    for g = 1:N_gates % + 1 for start gate
        % Id
        gates(g).order = g;

        % Gate size
        gates(g).radius           = 0.5; % Height, Width

        % Tolerances
        gates(g).time_tol         = 10;
        gates(g).position_tol     = 0.1;
        gates(g).velocity_tol     = 10;
        % This must be actually set smartly!
        gates(g).orientation_tol  = pi/8; % NOT SURE psi is pi to -pi
        gates(g).spin_tol         = 50*(2*pi/360); % angular velocity
    end

    u_hover = [13.7, 0, 0, 0];

    %% GATE 1
    % Drone states:
    gates(1).normal      = [0, 0, 1];
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
    gates(2).normal      = [0, 0, 1];
    gates(2).position    = [0, 0, 1];
    gates(2).velocity    = gates(2).normal;
    gates(2).orientation = zeros(1, 3); % must be normalized...
    gates(2).spin        = zeros(1, 3);
    

    gates(2).guess_time = 1;
    gates(2).time_min   = 0;
    gates(2).time_max   = gates(2).time_tol;

    gates(2).guess_state = [gates(2).position, ...
                            gates(2).velocity, ...
                            gates(2).orientation, ...
                            gates(2).spin];
    % The velocity dot normal must be higher than a threshold.
    % How does this translate into a tol constraint in 3D?
    % v_x * n_x + v_y * n_y + v_z * n_z > 0.2
    % 
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
    gates(3).normal      = [0, 0, 1];
    gates(3).position    = [0, 2, 2];
    gates(3).velocity    = gates(3).normal;
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

    %% GATE 4
    gates(4).normal      = [0, 0, 1];
    gates(4).position    = [1, 3, 1];
    gates(4).velocity    = gates(4).normal;
    gates(4).orientation = zeros(1, 3);
    gates(4).spin        = zeros(1, 3);
    

    gates(4).guess_time = 3;
    gates(4).time_min   = 0;
    gates(4).time_max   = gates(4).time_tol;

    gates(4).guess_state = [gates(4).position, ...
                            gates(4).velocity, ...
                            gates(4).orientation, ...
                            gates(4).spin];
    gates(4).state_min = [gates(4).position    - gates(4).position_tol, ...
                          gates(4).velocity    - gates(4).velocity_tol, ...
                          gates(4).orientation - gates(4).orientation_tol, ...
                          gates(4).spin        - gates(4).spin_tol];
    gates(4).state_max = [gates(4).position    + gates(4).position_tol, ...
                          gates(4).velocity    + gates(4).velocity_tol, ...
                          gates(4).orientation + gates(4).orientation_tol, ...
                          gates(4).spin        + gates(4).spin_tol];

    gates(4).guess_control = u_hover;

    %% GATE 5
    gates(5).normal      = [0, 0, 1];
    gates(5).position    = [2, 1, 2];
    gates(5).velocity    = gates(5).normal;
    gates(5).orientation = zeros(1, 3);
    gates(5).spin        = zeros(1, 3);
    gates(5).normal      = angle2rod(0, 0, 0);

    gates(5).guess_time = 4;
    gates(5).time_min   = 0;
    gates(5).time_max   = gates(5).time_tol;

    gates(5).guess_state = [gates(5).position, ...
                            gates(5).velocity, ...
                            gates(5).orientation, ...
                            gates(5).spin];
    gates(5).state_min = [gates(5).position    - gates(5).position_tol, ...
                          gates(5).velocity    - gates(5).velocity_tol, ...
                          gates(5).orientation - gates(5).orientation_tol, ...
                          gates(5).spin        - gates(5).spin_tol];
    gates(5).state_max = [gates(5).position    + gates(5).position_tol, ...
                          gates(5).velocity    + gates(5).velocity_tol, ...
                          gates(5).orientation + gates(5).orientation_tol, ...
                          gates(5).spin        + gates(5).spin_tol];

    gates(5).guess_control = u_hover;

    % Sanity check that there are as many gates as expected.
    assert(length(gates) == N_gates);
end


