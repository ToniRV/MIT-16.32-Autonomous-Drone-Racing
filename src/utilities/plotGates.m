function plotGates(gates)
%PLOTGATES plot gates in 3D
    for g = 1:length(gates)
        size_x = gates(g).size(1);
        size_y = gates(g).size(2);
        edges_3d_body = [size_x size_x -size_x -size_x;
                         size_y -size_y -size_y size_y;
                         0 0 0 0];
        
        phi   = gates(g).orientation(1);
        theta = gates(g).orientation(2);
        psi   = gates(g).orientation(3);
        
        [edge_x_tmp, edge_y_tmp, edge_z_tmp] = ...
            rotateBFtoGF(edges_3d_body(1, :),...
                         edges_3d_body(2, :),...
                         edges_3d_body(3, :), ...
                         phi, theta, psi);

        edge_x_tmp = gates(g).position(1) + edge_x_tmp;
        edge_y_tmp = gates(g).position(2) + edge_y_tmp;
        edge_z_tmp = gates(g).position(3) + edge_z_tmp;
        
        patch('xdata', edge_x_tmp, ...
              'ydata', edge_y_tmp, ...
              'zdata', edge_z_tmp, ...
              'facealpha', .9, 'facecolor', 'r');
    end
end

