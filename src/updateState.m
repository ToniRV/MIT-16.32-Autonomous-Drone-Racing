global Quad;

%% Update Velocities and Positions
% Calculating the Z velocity & position
Quad.State.Z_dot = Quad.Dynamics.Z_ddot*Quad.Ts + Quad.Dynamics.Z_dot;
Quad.State.Z = Quad.Dynamics.Z_dot*Quad.Ts + Quad.State.Z;
% Calculating the X velocity & position
Quad.State.X_dot = Quad.Dynamics.X_ddot*Quad.Ts + Quad.Dynamics.X_dot;
Quad.State.X = Quad.Dynamics.X_dot*Quad.Ts + Quad.State.X;
% Calculating the Y velocity & position
Quad.State.Y_dot = Quad.Dynamics.Y_ddot*Quad.Ts + Quad.Dynamics.Y_dot;
Quad.State.Y = Quad.Dynamics.Y_dot*Quad.Ts + Quad.State.Y;
% Calculating p,q,r
Quad.State.p = Quad.Dynamics.p_dot*Quad.Ts+Quad.State.p;
Quad.State.q = Quad.Dynamics.q_dot*Quad.Ts+Quad.State.q;
Quad.State.r = Quad.Dynamics.r_dot*Quad.Ts+Quad.State.r;
% Calculating angular velocity and position
Quad.State.phi = Quad.Dynamics.phi_dot*Quad.Ts + Quad.State.phi;
Quad.State.theta = Quad.Dynamics.theta_dot*Quad.Ts+Quad.State.theta;
Quad.State.psi = Quad.Dynamics.psi_dot*Quad.Ts+Quad.State.psi;

%% Update Plotting Variables
% Flip positive Z axis up for intuitive plotting
Quad.Z_plot(Quad.counter) = -Quad.State.Z;
Quad.Z_ref_plot(Quad.counter) = -Quad.Z_des;
% X plot
Quad.X_plot(Quad.counter) = Quad.State.X;
Quad.X_ref_plot(Quad.counter) = Quad.X_des;
% Y plot
Quad.Y_plot(Quad.counter) = Quad.State.Y;
Quad.Y_ref_plot(Quad.counter) = Quad.Y_des;
% Phi
Quad.phi_plot(Quad.counter) = Quad.State.phi;
Quad.phi_ref_plot(Quad.counter) = Quad.phi_des;
% Theta
Quad.theta_plot(Quad.counter) = Quad.State.theta;
Quad.theta_ref_plot(Quad.counter) = Quad.theta_des;
 % Psi
Quad.psi_plot(Quad.counter) = Quad.State.psi;
Quad.psi_ref_plot(Quad.counter) = Quad.psi_des;
