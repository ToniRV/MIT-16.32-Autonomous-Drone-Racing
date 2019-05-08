%---------------------------------------------%
% BEGIN: function continuous.m %
%---------------------------------------------%
function phaseout = continuous(input)
% Continuous function returns continuous signals
% - dynamics: x' = f(x, u, t)
% - path constraints (if any)
% - integrands (if any)
global Quad;

N_gates = input.auxdata.N_gates;

for p = 1:N_gates
    %% Input
    % Input State
    Quad.State = vectorToState(input.phase(p).state);
    
    % Input Control
    Quad.Control = vectorToControl(input.phase(p).control);
    
    %% Simulate dynamics
    % Nonlinear Dynamics given inputs and current state.
    nonlinearQuadrotorDynamics(Quad.State, Quad.Control);
 
    %% Output
    % Output dynamics
    phaseout(p).dynamics = dynamicsToVector(Quad.Dynamics);
    
end

%---------------------------------------------%
% END: function minCurve.m   %
%---------------------------------------------%