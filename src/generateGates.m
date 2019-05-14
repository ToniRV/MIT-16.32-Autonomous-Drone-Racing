function gates = generateGates(gates_data)
%GENERATEGATES generates a vector of gates.

    u_hover = [13.7, 0, 0, 0];

    %% GATE 1: start gate
    % This one is particular as we set the drone to start with 0
    % velocity.
    % Drone states:
    gates(1).normal      = normalize(gates_data(1).normal, 'norm');
    gates(1).position    = gates_data(1).position;
    gates(1).velocity    = zeros(1, 3);

    gates(1).guess_time = 0;

    gates(1).guess_control = u_hover;

    %% GATE 2
    gates(2).normal      = normalize(gates_data(2).normal, 'norm');
    gates(2).position    = gates_data(2).position;
    gates(2).velocity    = gates(2).normal;

    gates(2).guess_time = 1;

    gates(2).guess_control = u_hover;
    
    
    %% GATE 3
    gates(3).normal      = normalize(gates_data(3).normal, 'norm');
    gates(3).position    = gates_data(3).position;
    gates(3).velocity    = gates(3).normal;

    gates(3).guess_time = 2;

    gates(3).guess_control = u_hover;
    

    %% GATE 4
    gates(4).normal      = normalize([0, 0, -1], 'norm');
    gates(4).position    = [1, 3, 1];
    gates(4).velocity    = gates(4).normal;

    gates(4).guess_time = 3;

    gates(4).guess_control = u_hover;

    %% GATE 5
    gates(5).normal      = normalize([1, -1, 0], 'norm');
    gates(5).position    = [2, 1, 0];
    gates(5).velocity    = gates(5).normal;

    gates(5).guess_time = 4;

    gates(5).guess_control = u_hover;
    
    %% GATE 6
    gates(6).normal      = normalize([-1, -1, 0], 'norm');
    gates(6).position    = [2, -1, 0];
    gates(6).velocity    = gates(5).normal;

    gates(6).guess_time = 5;

    gates(6).guess_control = u_hover;
    %}
    
    %% Setup common gates properties.
    for g = 1:length(gates) % + 1 for start gate
        % Id
        gates(g).order = g;

        % Gate size
        gates(g).radius           = 0.5; % Height, Width

        % State guesses
        gates(g).orientation = zeros(1, 3);
        gates(g).spin        = zeros(1, 3);

        % Tolerances
        gates(g).time_tol         = 10;
        gates(g).position_tol     = 0.01;
        gates(g).velocity_tol     = 10;
        % This must be actually set smartly!
        gates(g).orientation_tol  = pi/2; % NOT SURE psi is pi to -pi
        gates(g).spin_tol         = 50*(2*pi/360); % angular velocity
        gates(g).vel_direction_tol   = 0.8; % Bound for dot prod of velocity direction to gate's normal
        gates(g).vel_norm_tol        = 9; % Lower bound for norm of velocity at gate's position for proper gate traversal

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
    
    % Fixed initial time for starting gate to 0.
    gates(1).time_min   = 0;
    gates(1).time_max   = 0;

    % Fixed initial state for starting gate to zero.
    gates(1).state_min   = [gates(1).position, gates(1).velocity, gates(1).orientation, gates(1).spin];
    gates(1).state_max   = [gates(1).position, gates(1).velocity, gates(1).orientation, gates(1).spin];
end


