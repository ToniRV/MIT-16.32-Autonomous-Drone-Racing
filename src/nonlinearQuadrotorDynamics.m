function nonlinearQuadrotorDynamics(State, Control)

global Quad;

% Cache calculations.
C_phi = cos(State.phi);
S_phi = sin(State.phi);
C_psi = cos(State.psi);
S_psi = sin(State.psi);
C_theta = cos(State.theta);
S_theta = sin(State.theta);
T_theta = tan(State.theta);

%% Linear Velocities
% Encoded in the quad state:
Quad.X_dot = State.X_dot;
Quad.Y_dot = State.Y_dot;
Quad.Z_dot = State.Z_dot;

%% Linear Accelerations
Quad.X_ddot = (-(C_phi * S_theta * C_psi + S_phi * S_psi) * Control.U1 - Quad.Kdx * State.x_dot) / Quad.m;
Quad.Y_ddot = (-(C_phi * S_psi * S_theta - C_psi * S_phi) * Control.U1 - Quad.Kdy * State.y_dot) / Quad.m;
Quad.Z_ddot = (-(C_phi * C_theta)                         * Control.U1 - Quad.Kdz * State.z_dot) / Quad.m + Quad.g;

%% Angular Velocities.
% World frame.
Quad.phi_dot   = State.p + State.r * (C_phi * T_theta) + State.q * (S_phi * T_theta);
Quad.theta_dot = C_phi * State.q - S_phi * State.r;
Quad.psi_dot   = (S_phi/C_theta) * State.q + (C_phi/C_theta) * State.r;

%% Angular Accelerations
% Body frame.
Quad.p_dot = (State.q*State.r*(Quad.Jy - Quad.Jz) - Quad.Jp*State.p*Quad.Obar + Quad.l*Control.U2)/Quad.Jx;
Quad.q_dot = (State.p*State.r*(Quad.Jz - Quad.Jx) + Quad.Jp*State.q*Quad.Obar + Quad.l*Control.U3)/Quad.Jy;
Quad.r_dot = (State.p*State.q*(Quad.Jx - Quad.Jy) + Control.U4)/Quad.Jz;