function initState()
% Init quadcopter state
  global Quad;
  Quad.State.X = 0;
  Quad.State.Y = 0;
  Quad.State.Z = 0;
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
