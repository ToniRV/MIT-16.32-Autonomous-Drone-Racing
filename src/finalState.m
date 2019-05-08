function state = gateState(gate_position, gate_orientation)
% Quadcopter state for given gate position and orientation.
  global Quad;
  Quad.State.X = gate_position.X;
  Quad.State.Y = gate_position.Y;
  Quad.State.Z = gate_position.Z;
  Quad.State.X_dot = 0;
  Quad.State.Y_dot = 0;
  Quad.State.Z_dot = 0;
  Quad.State.phi = 0;
  Quad.State.theta = 0;
  Quad.State.psi = 0;
  Quad.State.p = 0;
  Quad.State.q = 0;
  Quad.State.r = 0;
end
