%-------------------------------------------%
% BEGIN: function endpoint.m %
%-------------------------------------------%
function output = endpoint(input)
% Endpoint function returns discrete values
% I objective function, J
% I discrete constraints
% ? endpoint constraints
% ? state continuity across phases
% ? isoperimetric constraints

N_phases = input.auxdata.N_phases;
gates = input.auxdata.gates;

if N_phases > 1
    % Connect all phases together
    for p = 1:N_phases
        % Collect per phase initial/final Time and States
        % Time
        t0{p} = input.phase(p).initialtime;
        tf{p} = input.phase(p).finaltime;
        % States
        x0{p} = input.phase(p).initialstate;
        xf{p} = input.phase(p).finalstate;
    end

    for p = 1:N_phases - 1
        % Eventgroups
        output.eventgroup(p).event = [xf{p} - x0{p+1},...
                                      tf{p} - t0{p+1},...
                                      dot(normalize(xf{p}(4:6), 'norm'),...
                                          gates(p).normal)];
    end
end
    
output.objective = input.phase(end).finaltime;
end


%-------------------------------------------%
% END: function endpoint.m   %
%-------------------------------------------%

