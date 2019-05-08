function parametersQuad
%% Initialize Variables
global Quad;

% Simulation Parameters
Quad.init = 0;     % used in initilization
Quad.Ts = .01;     % Sampling time (100 Hz)
Quad.sim_time = 10; % Simulation time (seconds)
Quad.counter = 1;                      % the counter that holds the time value

% Plotting Variables
Quad.t_plot = [0 : Quad.Ts : Quad.sim_time - Quad.Ts];       % the time values
Quad.Xtemp = 0;     % Temp variables used rotating and plotting quadrotor
Quad.Ytemp = 0;     % Temp variables used rotating and plotting quadrotor
Quad.Ztemp = 0;     % Temp variables used rotating and plotting quadrotor

% Environmental Parametes
Quad.g = 9.81;     % Gravity (m/s^2)

% Quadrotor Physical Parameters
Quad.m = 1.4;      % Quadrotor mass (kg)
Quad.l = .56;     % Distance from the center of mass to the each motor (m)
Quad.t = .02;   %Thickness of the quadrotor's arms for drawing purposes (m)
Quad.rot_rad = .1;   %Radius of the propellor (m)
Quad.Kd = 1.3858e-6;    % Drag torque coeffecient (kg-m^2)

Quad.Kdx = 0.16481;    % Translational drag force coeffecient (kg/s)
Quad.Kdy = 0.31892;    % Translational drag force coeffecient (kg/s)
Quad.Kdz = 1.1E-6;    % Translational drag force coeffecient (kg/s)

Quad.Jx = .05;     % Moment of inertia about X axis (kg-m^2)
Quad.Jy = .05;     % Moment of inertia about Y axis (kg-m^2)
Quad.Jz = .24;    % Moment of inertia about Z axis (kg-m^2)

% Quadrotor Sensor Paramaters
Quad.GPS_freq = (1/Quad.Ts)/1;
Quad.X_error = .01;  %+/- m
Quad.Y_error = .01;  %+/- m
Quad.Z_error = .02;  %+/- m

Quad.x_acc_bias = 0.16594;  % m/s^2
Quad.x_acc_sd = 0.0093907;
Quad.y_acc_bias = 0.31691;  % m/s^2
Quad.y_acc_sd = 0.011045;
Quad.z_acc_bias = -8.6759;  % m/s^2
Quad.z_acc_sd = 0.016189;

Quad.x_gyro_bias = 0.00053417;  % rad/s
Quad.x_gyro_sd = 0.00066675;
Quad.y_gyro_bias = -0.0011035;  % rad/s
Quad.y_gyro_sd = 0.00053642;
Quad.z_gyro_bias = 0.00020838;  % rad/s
Quad.z_gyro_sd = 0.0004403;

Quad.ground_truth = 1;  % Use perfect sensor measurements
Quad.sensor_unfiltered = 0; % Use sensor errors, no filter
Quad.sensor_kf = 0;     % Use sensor error, Kalman Filter

% Motor Parameters
Quad.KT = 1.3328e-5;    % Thrust force coeffecient (kg-m)
Quad.Jp = 0.044;     % Moment of Intertia of the rotor (kg-m^2)
Quad.max_motor_speed = 925; % motors upper limit (rad/s)
Quad.min_motor_speed = 0; %-1*((400)^2); % motors lower limit (can't spin in reverse)

Quad.Obar = 0;     % sum of motor speeds (O1-O2+O3-O4, N-m)
Quad.O1 = 0;       % Front motor speed (raidans/s)
Quad.O2 = 0;       % Right motor speed (raidans/s)
Quad.O3 = 0;       % Rear motor speed (raidans/s)
Quad.O4 = 0;       % Left motor speed (raidans/s)

%Translational Positions
Quad.X = 0;        % Initial position in X direction GF (m)
Quad.Y = 0;        % Initial position in Y direction GF (m)
Quad.Z = 0;        % Initial position in Z direction GF (m)
Quad.X_BF = 0;     % Initial position in X direction BF (m)
Quad.Y_BF = 0;     % Initial position in Y direction BF (m)
Quad.Z_BF = 0;     % Initial position in the Z direction BF (m)

%Translational Velocities
Quad.X_dot = 0;    % Initial velocity in X direction GF (m/s)
Quad.Y_dot = 0;    % Initial velocity in Y direction GF (m/s)
Quad.Z_dot = 0;    % Initial velocity in Z direction GF (m/s)
Quad.X_dot_BF = 0;    % Initial velocity in X direction BF (m/s)
Quad.Y_dot_BF = 0;    % Initial velocity in Y direction BF (m/s)
Quad.Z_dot_BF = 0;    % Initial velocity in Y direction BF (m/s)

%Angular Positions
Quad.phi = 0;      % Initial phi value (rotation about X GF, roll,  radians)
Quad.theta = 0;    % Initial theta value (rotation about Y GF, pitch, radians)
Quad.psi = 0;      % Initial psi value (rotation about Z GF, yaw, radians)

%Angular Velocities
Quad.p = 0;        % Initial p value (angular rate rotation about X BF, radians/s)
Quad.q = 0;        % Initial q value (angular rate rotation about Y BF, radians/s)
Quad.r = 0;        % Initial r value (angular rate rotation about Z BF, radians/s)

% Desired variables
Quad.X_des_GF = 1;         % desired value of X in Global frame
Quad.Y_des_GF = 1;         % desired value of Y in Global frame
Quad.Z_des_GF = 1;         % desired value of Z in Global frame
Quad.X_des = 0;            % desired value of X in Body frame
Quad.Y_des = 0;            % desired value of Y in Body frame
Quad.Z_des = 0;            % desired value of Z in Body frame

Quad.phi_des = 0;          % desired value of phi (radians)
Quad.theta_des = 0;        % desired value of theta (radians)
Quad.psi_des = pi/6;          % desired value of psi (radians)

% Measured variables
Quad.X_meas = 0;
Quad.Y_meas = 0;
Quad.Z_meas = 0;
Quad.phi_meas = 0;
Quad.theta_meas = 0;
Quad.psi_meas = 0;

% Disturbance Variables
Quad.Z_dis = 0;            % Disturbance in Z direction
Quad.X_dis = 0;            % Disturbance in X direction
Quad.Y_dis = 0;            % Ddisturbance in Y direction
Quad.phi_dis = 0;            % Disturbance in Yaw direction
Quad.theta_dis = 0;            % Disturbance in Pitch direction
Quad.psi_dis = 0;            % Disturbance in Roll direction

% Control Inputs
Quad.U1 = 0;       % Total thrust (N)
Quad.U2 = 0;       % Torque about X axis BF (N-m)
Quad.U3 = 0;       % Torque about Y axis BF (N-m)
Quad.U4 = 0;       % Torque about Z axis BF (N-m)

%% PID parameters
Quad.X_KP = .35;          % KP value in X position control
Quad.X_KI = .25;            % KI value in X position control
Quad.X_KD = -.35;         % KD value in X position control
Quad.X_KI_lim = .25;         % Error to start calculating integral term

Quad.Y_KP = .35;          % KP value in Y position control
Quad.Y_KI = .25;            % KI value in Y position control
Quad.Y_KD = -.35;         % KD value in Y position control
Quad.Y_KI_lim = .25;         % Error to start calculating integral term

Quad.Z_KP = 10/1.7;    % KP value in altitude control
Quad.Z_KI = 0*3;    % KI value in altitude control
Quad.Z_KD = -10/1.980;  % KD value in altitude control
Quad.Z_KI_lim = .25;         % Error to start calculating integral term

Quad.phi_KP = 4.5;      % KP value in roll control 2
Quad.phi_KI = 0;       % KI value in roll control   1
Quad.phi_KD = 0;     % KD value in roll control  -.5
Quad.phi_max = pi/4;   % Maximum roll angle commanded
Quad.phi_KI_lim = 2*(2*pi/360);  % Error to start calculating integral

Quad.theta_KP = 4.5;    % KP value in pitch control 2
Quad.theta_KI = 0;     % KI value in pitch control 1
Quad.theta_KD = 0;   % KD value in pitch control -.5
Quad.theta_max = pi/4; % Maximum pitch angle commanded
Quad.theta_KI_lim = 2*(2*pi/360);  % Error to start calculating integral

Quad.psi_KP = 10;     % KP value in yaw control
Quad.psi_KI = 0;     % KI value in yaw control .75
Quad.psi_KD = 0;     % KD value in yaw control -.5
Quad.psi_KI_lim = 8*(2*pi/360);  % Error to start calculating integral

Quad.p_KP = 2.7;    % KP value in pitch control 2
Quad.p_KI = 1;     % KI value in pitch control
Quad.p_KD = -.01;   % KD value in pitch control -.5
Quad.p_max = 50*(2*pi/360); % Maximum pitch angle commanded
Quad.p_KI_lim = 10*(2*pi/360);  % Error to start calculating integral

Quad.q_KP = 2.7;    % KP value in pitch control
Quad.q_KI = 1;     % KI value in pitch control
Quad.q_KD = -.01;   % KD value in pitch control -.5
Quad.q_max = 50*(2*pi/360); % Maximum pitch angle commanded
Quad.q_KI_lim = 10*(2*pi/360);  % Error to start calculating integral

Quad.r_KP = 2.7;    % KP value in pitch control
Quad.r_KI = 1;     % KI value in pitch control
Quad.r_KD = -.01;   % KD value in pitch control
Quad.r_max = 50*(2*pi/360); % Maximum pitch angle commanded
Quad.r_KI_lim = 10*(2*pi/360);  % Error to start calculating integral

%% State Limits
% Absolute Pose [m]
Quad.X_min = -10;
Quad.X_max = 10;
Quad.Y_min = -10;
Quad.Y_max = 10;
Quad.Z_min = -10;
Quad.Z_max = 10;

% Linear Velocity [m/s]
Quad.X_dot_min = -10;
Quad.X_dot_max = 10;
Quad.Y_dot_min = -10;
Quad.Y_dot_max = 10;
Quad.Z_dot_min = -10;
Quad.Z_dot_max = 10;

% Angular position [rad]
Quad.phi_min = pi/2;
Quad.phi_max = -pi/2;
Quad.theta_min = pi/2;
Quad.theta_max = -pi/2;
Quad.psi_min = pi/2;
Quad.psi_max = -pi/2;

% Angular rates [rad/s]
Quad.p_min = -Quad.p_max;
Quad.q_min = -Quad.q_max;
Quad.r_min = -Quad.r_max;


% Control Limits (update values)
Quad.U1_max = 43.5;   % Quad.KT*4*Quad.max_motor_speed^2
Quad.U1_min = 0;      %
Quad.U2_max = 6.25;  % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U2_min = -6.25; % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U3_max = 6.25;  % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U3_min = -6.25; % Quad.KT*Quad.l*Quad.max_motor_speed^2
Quad.U4_max = 2.25; % Quad.Kd*2*Quad.max_motor_speed^2
Quad.U4_min = -2.25;% Quad.Kd*2*Quad.max_motor_speed^2
end
