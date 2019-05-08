%-------------------------------------------%
% BEGIN: function minCurveEndpoint.m %
%-------------------------------------------%
function output = endpoint(input)
% Endpoint function returns discrete values
% I objective function, J
% I discrete constraints
% ? endpoint constraints
% ? state continuity across phases
% ? isoperimetric constraints

N_gates = input.auxdata.N_gates;

if N_gates > 1
    % Connect all phases together
    for p = 1:N_gates
        % Collect per phase initial/final Time and States
        % Time
        t0{p} = input.phase(p).initialtime;
        tf{p} = input.phase(p).finaltime;
        % States
        x0{p} = input.phase(p).initialstate;
        xf{p} = input.phase(p).finalstate;
    end

    for p = 1:N_gates-1
        % Eventgroups
        output.eventgroup(p).event = [xf{p} - x0{p+1},...
                                      tf{p} - t0{p+1}];
    end
end
    
output.objective = input.phase(end).finaltime;
% output.objective = input.phase(1).integral + ...
%                    input.phase(2).integral + ...
%                    input.phase(3).integral;
end


%-------------------------------------------%
% END: function minCurveEndpoint.m   %
%-------------------------------------------%

