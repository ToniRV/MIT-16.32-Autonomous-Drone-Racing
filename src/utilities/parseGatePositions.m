function [gates, max_pos, min_pos] = parseGatePositions()
%PARSEGATEPOSITIONS Parses gate positions given from a yaml file,
%structured as:
% Gate1:
%   nominal_location: [xyz_corner1,  xyz_corner2, xyz_corner3, xyz_corner4]
% Gate2:
%   nominal_location: [xyz_corner1,  xyz_corner2, xyz_corner3, xyz_corner4]
% For example:
% Gate1:
%   nominal_location: [[-0.009000421, -32.9505, 3.071861], [-0.009000659, -34.8755, 3.071861], [-0.009000659, -34.8755, 1.134362], [-0.009000421, -32.9505, 1.134362]]
%   perturbation_bound: [2.5, 2.5, 5.0]
% Gate13:
%   nominal_location: [[1.237332, 9.001728, 2.9625], [3.162332, 9.001728, 2.9625], [3.162332, 9.001728, 1.025], [1.237332, 9.001728, 1.025]]
%   perturbation_bound: [2.5, 2.5, 5.0]

    % Parse gate's positions:
    X = YAML.read("./config/gate_locations.yaml");
    fn = fieldnames(X);
    max_pos = -inf * ones(1,3);
    min_pos = inf * ones(1,3);
    for k = 1:numel(fn)
        % Get gate's position.
        gates(k).position = mean(X.(fn{k}).nominal_location) / 100;
        
        % Find min and max gates positions to setup state bounds later.
        for i = 1:length(gates(k).position)
            gate_position_component_i = gates(k).position(i);
            if max_pos(i) < gate_position_component_i
                max_pos(i) = gate_position_component_i;
            end
            if min_pos(i) > gate_position_component_i
                min_pos(i) = gate_position_component_i;
            end
        end
          
        % How is the sign of the normal given??
        gates(k).normal = normalize(...
            cross(X.(fn{k}).nominal_location(2,:) - X.(fn{k}).nominal_location(1,:),...
                  X.(fn{k}).nominal_location(3,:) - X.(fn{k}).nominal_location(1,:)), 'norm');
              
    end
end

