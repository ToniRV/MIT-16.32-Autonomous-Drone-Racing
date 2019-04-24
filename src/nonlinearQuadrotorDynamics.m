function nonlinearQuadrotorDynamics(State. Control)

global Quad;

% Cache calculations.
C_phi = cos(State.phi);
S_phi = sin(State.phi);
C_psi = cos(State.psi);
S_psi = sin(State.psi);
C_theta = cos(State.theta);
S_theta = sin(State.theta);

%% Linear Accelerations
Quad.X_ddot = (-[C_phi*S_theta*C_psi + S_phi*S_psi]*Control.U1 - Quad.Kdx*State.x_dot) / Quad.m;
Quad.Y_ddot = (-[C_phi*S_psi*S_theta - C_psi*S_phi]*Control.U1 - Quad.Kdy*State.y_dot) / Quad.m;
Quad.Z_ddot = (-[C_phi*C_theta]*Control.U1 - Quad.Kdz*State.z_dot) / Quad.m + Quad.g;

%% Angular Accelerations
% Body frame.
Quad.p_dot = (State.q*Quad.r*(Quad.Jy - Quad.Jz) - Quad.Jp*Quad.p*Quad.Obar + Quad.l*Quad.U2)/Quad.Jx;
Quad.q_dot = (State.p*Quad.r*(Quad.Jz - Quad.Jx) + Quad.Jp*Quad.q*Quad.Obar + Quad.l*Quad.U3)/Quad.Jy;
Quad.r_dot = (Quad.p*Quad.q*(Quad.Jx - Quad.Jy) + Quad.U4)/Quad.Jz;

% World frame.
Quad.phi_dot   = Quad.p + sin(Quad.phi)*tan(Quad.theta)*Quad.q + cos(Quad.phi)*tan(Quad.theta)*Quad.r;
Quad.theta_dot = cos(Quad.phi)*Quad.q - sin(Quad.phi)*Quad.r;
Quad.psi_dot   = sin(Quad.phi)/cos(Quad.theta)*Quad.q + cos(Quad.phi)/cos(Quad.theta)*Quad.r;

%% Correct angular accelerations
a_1 = (Quad.Jy - Quad.Jz) / Quad.Jx
a_2 = (Quad.Jz - Quad.Jx) / Quad.Jy
a_3 = (Quad.Jx - Quad.Jy) / Quad.Jz

b_1 = l / Quad.Jx
b_2 = l / Quad.Jy
b_3 = l / Quad.Jz

Quad.phi_dot   = sin(Quad.phi)*tan(Quad.theta)*Quad.q + cos(Quad.phi)*tan(Quad.theta)*Quad.r;
Quad.theta_dot = cos(Quad.phi)*Quad.q - sin(Quad.phi)*Quad.r;
Quad.psi_dot   = sin(Quad.phi)/cos(Quad.theta)*Quad.q + cos(Quad.phi)/cos(Quad.theta)*Quad.r;
