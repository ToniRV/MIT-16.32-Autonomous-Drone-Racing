function plotVelocityCones(gates)
%PLOTVELOCITYCONES Summary of this function goes here
%   Detailed explanation goes here
    % Cone parameters for the velocity constraints.
    n=20;
    cyl_color = 'b';
    closed = 0;
    lines = 0;
    alpha = 0.25;
    
   for g = 1:length(gates)
        X2 = gates(g).position + gates(g).normal;
        r=[0 tan(acos(gates(g).vel_normal_tol))];
        
        [Cone, ~, ~] = plotCone(gates(g).position, X2, r, n, cyl_color, closed, lines);
        set(Cone,'FaceAlpha', alpha)
    end
end

