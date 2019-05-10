function plotGates(gates)
%PLOTGATES plot gates in 3D
    for g = 1:length(gates)
        plotCircle3D(gates(g).position, ...
                     gates(g).normal,...
                     gates(g).radius, 'r')
    end
end

