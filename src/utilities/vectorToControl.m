function control = vectorToControl(vector)
%VECTORTOCONTROL Transforms a gpops vector to a control vector
  control.U1 = vector(:, 1);
  control.U2 = vector(:, 2);
  control.U3 = vector(:, 3);
  control.U4 = vector(:, 4);
end

