%---------------------------------------------%
% BEGIN: function continuous.m %
%---------------------------------------------%
function phaseout = continuous(input)
% Continuous function returns continuous signals
% - dynamics: x' = f(x, u, t)
% - path constraints (if any)
% - integrands (if any)
global Quad;

N_phases = input.auxdata;

for p = 1:N_phases
    %% Simulate dynamics
    % Nonlinear Dynamics given inputs and current state.
    nonlinearQuadrotorDynamics(vectorToState(input.phase(p).state), ...
                               vectorToControl(input.phase(p).control));
 
    %% Output
    % Output dynamics
    phaseout(p).dynamics = dynamicsToVector(Quad.Dynamics);
    
end

%---------------------------------------------%
% END: function minCurve.m   %
%---------------------------------------------%