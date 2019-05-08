function state = vectorToState(vector)
  % VECTORTOSTATE Transforms a gpops vector to a state vector
  state.X     = vector(:, 1);
  state.Y     = vector(:, 2);
  state.Z     = vector(:, 3);
  state.X_dot = vector(:, 4);
  state.Y_dot = vector(:, 5);
  state.Z_dot = vector(:, 6);
  state.phi   = vector(:, 7);
  state.theta = vector(:, 8);
  state.psi   = vector(:, 9);
  state.p     = vector(:, 10);
  state.q     = vector(:, 11);
  state.r     = vector(:, 12);
end

