% Wil Selby
% Washington, DC
% May 30, 2015

% This function draws the quadrotor model by rotating the original Body
% Frame coordinates used to define the quadrotor in plot_quad_model.m into
% Global Frame coordinates. It then moves the quadrotor to the current
% Global Position output from the quadrotor dynamics. In our NED coordinate
% frame, positive Z is negative Z in MATLAB and positive Y in negative Y in
% MATLAB. This is reflected iwth the - in the set function.

%% Draw the quadrotor arms and rotors
global Quad

[Quad.Xtemp, Quad.Ytemp, Quad.Ztemp] = ...
rotateBFtoGF(Quad.X_armX, Quad.X_armY, Quad.X_armZ, ...
             Quad.State.phi, Quad.State.theta, Quad.State.psi);
set(Quad.X_arm, ...
    'xdata',   Quad.Xtemp + Quad.State.X, ...
    'ydata', -(Quad.Ytemp + Quad.State.Y),...
    'zdata', -(Quad.Ztemp + Quad.State.Z))

[Quad.Xtemp,Quad.Ytemp,Quad.Ztemp]=rotateBFtoGF(Quad.Y_armX,Quad.Y_armY,Quad.Y_armZ,Quad.State.phi,Quad.State.theta,Quad.State.psi);
set(Quad.Y_arm,'xdata',Quad.Xtemp+Quad.State.X,'ydata',-(Quad.Ytemp+Quad.State.Y),'zdata',-(Quad.Ztemp+Quad.State.Z))

[Quad.Xtemp,Quad.Ytemp,Quad.Ztemp]=rotateBFtoGF(Quad.Motor1X,Quad.Motor1Y,Quad.Motor1Z,Quad.State.phi,Quad.State.theta,Quad.State.psi);
set(Quad.Motor1,'xdata',Quad.Xtemp+Quad.State.X,'ydata',-(Quad.Ytemp+Quad.State.Y),'zdata',-(Quad.Ztemp+Quad.State.Z-2*Quad.t))

[Quad.Xtemp,Quad.Ytemp,Quad.Ztemp]=rotateBFtoGF(Quad.Motor2X,Quad.Motor2Y,Quad.Motor2Z,Quad.State.phi,Quad.State.theta,Quad.State.psi);
set(Quad.Motor2,'xdata',Quad.Xtemp+Quad.State.X,'ydata',-(Quad.Ytemp+Quad.State.Y),'zdata',-(Quad.Ztemp+Quad.State.Z-2*Quad.t))

[Quad.Xtemp,Quad.Ytemp,Quad.Ztemp]=rotateBFtoGF(Quad.Motor3X,Quad.Motor3Y,Quad.Motor3Z,Quad.State.phi,Quad.State.theta,Quad.State.psi);
set(Quad.Motor3,'xdata',Quad.Xtemp+Quad.State.X,'ydata',-(Quad.Ytemp+Quad.State.Y),'zdata',-(Quad.Ztemp+Quad.State.Z-2*Quad.t))

[Quad.Xtemp,Quad.Ytemp,Quad.Ztemp]=rotateBFtoGF(Quad.Motor4X,Quad.Motor4Y,Quad.Motor4Z,Quad.State.phi,Quad.State.theta,Quad.State.psi);
set(Quad.Motor4,'xdata',Quad.Xtemp+Quad.State.X,'ydata',-(Quad.Ytemp+Quad.State.Y),'zdata',-(Quad.Ztemp+Quad.State.Z-2*Quad.t))


