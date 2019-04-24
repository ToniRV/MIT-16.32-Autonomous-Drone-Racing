%---------------------------------------------%
% BEGIN: function minCurve.m %
%---------------------------------------------%
function phaseout = continuous(input)
% PHASE 1
v                 = input.phase(1).state(:,2);
u                 = input.phase(1).control;
phaseout(1).dynamics = [v, -v + u];
phaseout(1).integrand = u.*v;

% PHASE 2
v                 = input.phase(2).state(:,2);
u                 = v;
phaseout(2).dynamics = [v, -v + u];
phaseout(2).integrand = u.*v;

% PHASE 3
v                 = input.phase(3).state(:,2);
u                 = input.phase(3).control;
phaseout(3).dynamics = [v, -v + u];
phaseout(3).integrand = u.*v;


%---------------------------------------------%
% END: function minCurve.m   %
%---------------------------------------------%