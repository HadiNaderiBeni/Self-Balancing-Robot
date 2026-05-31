%% ET_LQR_Setup.m
% Practical event-triggered LQR setup for balancing robot
% Run after Nominal_Tracking.m

Nominal_Tracking;

%% Use nominal LQR gain from your code
K_ET_LQR = K_Nom_LQR_Br;

%% Closed-loop discrete-time matrix
Phi_cl_ET = Phi - Gamma*K_ET_LQR;

%% Lyapunov matrix for event trigger
% You can use eye(4) for a simple local Lyapunov matrix,
% or use your LQR state weight Q from Nominal_Tracking.m.
Q_ET = eye(4);

P_ET = dlyap(Phi_cl_ET.', Q_ET);
P_ET = 0.5*(P_ET + P_ET.');   % numerical symmetrization

%% Event-trigger parameters
sigma_ET = 0.10;              % smaller = safer, more events
lambda_eta_ET = 1.5;          % eta decay rate for V-A strategy

Tmin_ET = 0.03;               % minimum inter-event time [s]
minSteps_ET = max(1, round(Tmin_ET/Ts));

%% Output saturation
% Since your model uses voltage command ua [V], keep ET output in volts.
uMax_ET = drv.Vbus;           % 11.1 V from BalRob_Params

%% Trigger mode
% 1 = V-A threshold-variable trigger
% 2 = V-C simple W(e) >= V(x)
mode_ET = 2;                  % start with simple mode first

%% Optional: reference and disturbance kept zero for first test
gam_ref_ET = 0;
dist_ET = 0;

%first test
%%%mode_ET = 2;
%%%sigma_ET = 0.10;
%%%Tmin_ET = 0.03;

%second test
%mode_ET = 1;
%sigma_ET = 0.15;
%lambda_eta_ET = 1.5;