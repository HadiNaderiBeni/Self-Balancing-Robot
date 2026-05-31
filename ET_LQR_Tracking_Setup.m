%% ET_LQR_Tracking_Setup.m
% Practical event-triggered LQR setup for CONSTANT reference tracking
% Balancing robot state:
%   x = [gamma; theta; gamma_dot; theta_dot]
% units:
%   [rad; rad; rad/s; rad/s]
%
% Run this before Simulink.

Nominal_Tracking;

%% Use nominal LQR gain from your code
K_ET_LQR = K_Nom_LQR_Br;

%% Tracking feedforward terms from Nominal_Tracking.m
% For a constant reference r = gam_ref_rad:
%
%   x_ss = Nx_ET*r
%   ua_ss = Nu_ET*r
%
% Tracking control law:
%
%   ua = Nu_ET*r - K_ET_LQR*(x - Nx_ET*r)

Nx_ET = Nx;       % 4x1
Nu_ET = Nu;       % scalar

%% Closed-loop discrete-time matrix for tracking-error dynamics
% Define:
%   x_tilde = x - Nx_ET*r
%
% For constant r:
%   x_tilde[k+1] = (Phi - Gamma*K_ET_LQR)*x_tilde[k]

Phi_cl_ET = Phi - Gamma*K_ET_LQR;

%% Lyapunov matrix for event trigger
% This Lyapunov function is applied to x_tilde, not raw x:
%
%   V = x_tilde' * P_ET * x_tilde

Q_ET = eye(4);

P_ET = dlyap(Phi_cl_ET.', Q_ET);
P_ET = 0.5*(P_ET + P_ET.');   % numerical symmetrization

%% Event-trigger parameters
sigma_ET = 0.10;              % smaller = safer, more events
lambda_eta_ET = 1.5;          % eta decay rate for V-A strategy

Tmin_ET = 0.03;               % minimum inter-event time [s]
minSteps_ET = max(1, round(Tmin_ET/Ts));

%% Output saturation
% Your model/controller input is motor armature voltage ua [V]
uMax_ET = drv.Vbus;           % normally 11.1 V

%% Trigger mode
% 1 = V-A threshold-variable trigger: W(e) >= max(V(x_tilde), eta)
% 2 = V-C simple trigger:            W(e) >= V(x_tilde)

mode_ET = 2;                  % start with simple mode first

%% Constant reference
% IMPORTANT:
% Use radians here.
%
% Example:
%   0 deg  -> 0
%   5 deg  -> 5*deg2rad
%   10 deg -> 10*deg2rad

gam_ref_ET = 0;               % start with zero reference
% gam_ref_ET = 5*deg2rad;     % later test: 5 deg reference

%% Disturbance kept zero for first tracking test
dist_ET = 0;

%% Suggested first tracking test
% mode_ET = 2;
% sigma_ET = 0.10;
% Tmin_ET = 0.03;
% minSteps_ET = max(1, round(Tmin_ET/Ts));

%% Suggested second tracking test
% mode_ET = 1;
% sigma_ET = 0.15;
% lambda_eta_ET = 1.5;