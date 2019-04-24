%-------------------------------------------%
% BEGIN: function minCurveEndpoint.m %
%-------------------------------------------%
function output = endpoint(input)

% PHASE 1
t0{1} = input.phase(1).initialtime;
tf{1} = input.phase(1).finaltime;
x0{1} = input.phase(1).initialstate;
xf{1} = input.phase(1).finalstate;

% PHASE 2
t0{2} = input.phase(2).initialtime;
tf{2} = input.phase(2).finaltime;
x0{2} = input.phase(2).initialstate;
xf{2} = input.phase(2).finalstate;

% PHASE 3
t0{3} = input.phase(3).initialtime;
tf{3} = input.phase(3).finaltime;
x0{3} = input.phase(3).initialstate;
xf{3} = input.phase(3).finalstate;

% Eventgroups
output.eventgroup(1).event = [xf{1}(1:2) - x0{2}(1:2), tf{1} - t0{2}];
output.eventgroup(2).event = [xf{2}(1:2) - x0{3}(1:2), tf{2} - t0{3}];

output.objective = input.phase(1).integral + ...
                   input.phase(2).integral + ...
                   input.phase(3).integral;

end


%-------------------------------------------%
% END: function minCurveEndpoint.m   %
%-------------------------------------------%

