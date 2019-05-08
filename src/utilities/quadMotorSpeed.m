
function quadMotorSpeed
% Calculates motor speeds from Quad control inputs.

global Quad

% Calculate motor speeds (rad/s)^2
w1 = Quad.Control.U1/(4*Quad.KT) + Quad.Control.U3/(2*Quad.KT*Quad.l) + Quad.Control.U4/(4*Quad.Kd);
w2 = Quad.Control.U1/(4*Quad.KT) - Quad.Control.U2/(2*Quad.KT*Quad.l) - Quad.Control.U4/(4*Quad.Kd);
w3 = Quad.Control.U1/(4*Quad.KT) - Quad.Control.U3/(2*Quad.KT*Quad.l) + Quad.Control.U4/(4*Quad.Kd);
w4 = Quad.Control.U1/(4*Quad.KT) + Quad.Control.U2/(2*Quad.KT*Quad.l) - Quad.Control.U4/(4*Quad.Kd);

Quad.O1 = sqrt(w1);    % Front M
Quad.O2 = sqrt(w2);    % Right M
Quad.O3 = sqrt(w3);    % Rear M
Quad.O4 = sqrt(w4);    % Left M

Quad.O1_plot(Quad.counter) = Quad.O1;
Quad.O2_plot(Quad.counter) = Quad.O2;
Quad.O3_plot(Quad.counter) = Quad.O3;
Quad.O4_plot(Quad.counter) = Quad.O4;

end