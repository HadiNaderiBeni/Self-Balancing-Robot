%% run_balrob_VD_VE_trigger_simulation.m
% V-D and V-E event-triggered LQR simulation for the self-balancing robot.
%
% Uses:
%   BalRob_Params.m
%   Nominal_Tracking.m
%
% Compares:
%   1) V-D practical fixed-error/deadband trigger
%   2) V-E practical Lyapunov-envelope trigger
%   3) V-A threshold-variable trigger
%   4) V-C ISS/Lyapunov trigger
%   5) Periodic sampled LQR

clear; clc; close all;

%% Load robot model and LQR
Nominal_Tracking;

K = K_Nom_LQR_Br;

%% Simulation parameters
Tsim = 8.0;              % [s]
dt   = 1e-3;             % integration step [s]
N    = round(Tsim/dt) + 1;
t    = (0:N-1)*dt;

x_init = x0(:);          % [0; 5 deg; 0; 0]
uMax = drv.Vbus;         % voltage saturation [V]

%% Lyapunov matrix
Acl = A - B*K;
P = lyap(Acl.', eye(4));
P = 0.5*(P + P.');

%% Common trigger settings
Tmin = 0.005;            % minimum inter-event time [s]
mu = 0.02;               % Lyapunov scaling, as in your previous slides

%% V-D settings
eu_threshold_D = 0.20;   % [V]
rho_D = eu_threshold_D^2;

%% V-E settings
lambda_eta_VE = 1.50;    % envelope decay rate
alpha_VE      = 1.00;    % derivative guard: Vdot >= -alpha*V
V_floor_VE    = 1e-7;    % suppress useless near-origin triggering
eta0_VE       = mu*x_init.'*P*x_init;

%% V-A and V-C settings for comparison
lambda_eta_VA = 10.0;
eta0_VA       = mu*x_init.'*P*x_init;

%% Run simulations
res_VD = simulate_ET_robot(A, B, K, P, x_init, dt, Tsim, uMax, Tmin, mu, ...
                           "VD", rho_D, lambda_eta_VE, alpha_VE, V_floor_VE, eta0_VE, ...
                           lambda_eta_VA, eta0_VA, Ts);

res_VE = simulate_ET_robot(A, B, K, P, x_init, dt, Tsim, uMax, Tmin, mu, ...
                           "VE", rho_D, lambda_eta_VE, alpha_VE, V_floor_VE, eta0_VE, ...
                           lambda_eta_VA, eta0_VA, Ts);

res_VA = simulate_ET_robot(A, B, K, P, x_init, dt, Tsim, uMax, Tmin, mu, ...
                           "VA", rho_D, lambda_eta_VE, alpha_VE, V_floor_VE, eta0_VE, ...
                           lambda_eta_VA, eta0_VA, Ts);

res_VC = simulate_ET_robot(A, B, K, P, x_init, dt, Tsim, uMax, Tmin, mu, ...
                           "VC", rho_D, lambda_eta_VE, alpha_VE, V_floor_VE, eta0_VE, ...
                           lambda_eta_VA, eta0_VA, Ts);

res_Per = simulate_ET_robot(A, B, K, P, x_init, dt, Tsim, uMax, Tmin, mu, ...
                            "Periodic", rho_D, lambda_eta_VE, alpha_VE, V_floor_VE, eta0_VE, ...
                            lambda_eta_VA, eta0_VA, Ts);

%% Print results
fprintf('\nUpdate counts over %.2f s:\n', Tsim);
fprintf('  V-D fixed threshold  : %d events\n', res_VD.eventCount);
fprintf('  V-E Lyap envelope    : %d events\n', res_VE.eventCount);
fprintf('  V-A threshold var.   : %d events\n', res_VA.eventCount);
fprintf('  V-C W>=V             : %d events\n', res_VC.eventCount);
fprintf('  Periodic             : %d updates\n', res_Per.eventCount);

fprintf('\nUpdate reduction versus periodic:\n');
fprintf('  V-D: %.2f %%\n', 100*(1 - res_VD.eventCount/res_Per.eventCount));
fprintf('  V-E: %.2f %%\n', 100*(1 - res_VE.eventCount/res_Per.eventCount));
fprintf('  V-A: %.2f %%\n', 100*(1 - res_VA.eventCount/res_Per.eventCount));
fprintf('  V-C: %.2f %%\n', 100*(1 - res_VC.eventCount/res_Per.eventCount));

%% Tilt angle
figure('Name','Tilt angle comparison');
plot(t, res_VD.x(2,:)*rad2deg, 'LineWidth', 1.4); hold on;
plot(t, res_VE.x(2,:)*rad2deg, 'LineWidth', 1.4);
plot(t, res_VA.x(2,:)*rad2deg, '--', 'LineWidth', 1.2);
plot(t, res_VC.x(2,:)*rad2deg, '-.', 'LineWidth', 1.2);
plot(t, res_Per.x(2,:)*rad2deg, ':', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('\theta [deg]');
title('Tilt angle comparison');
legend('V-D','V-E','V-A','V-C','Periodic','Location','best');

%% Wheel angle
figure('Name','Wheel angle comparison');
plot(t, res_VD.x(1,:), 'LineWidth', 1.4); hold on;
plot(t, res_VE.x(1,:), 'LineWidth', 1.4);
plot(t, res_VA.x(1,:), '--', 'LineWidth', 1.2);
plot(t, res_VC.x(1,:), '-.', 'LineWidth', 1.2);
plot(t, res_Per.x(1,:), ':', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('\gamma [rad]');
title('Wheel angle comparison');
legend('V-D','V-E','V-A','V-C','Periodic','Location','best');

%% Control input
figure('Name','Control input comparison');
plot(t, res_VD.u, 'LineWidth', 1.4); hold on;
plot(t, res_VE.u, 'LineWidth', 1.4);
plot(t, res_VA.u, '--', 'LineWidth', 1.2);
plot(t, res_VC.u, '-.', 'LineWidth', 1.2);
plot(t, res_Per.u, ':', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('u_a [V]');
title('Control input comparison');
legend('V-D','V-E','V-A','V-C','Periodic','Location','best');

%% V-E envelope
figure('Name','V-E Lyapunov envelope');
plot(t, res_VE.V, 'LineWidth', 1.5); hold on;
plot(t, res_VE.eta, '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Threshold values');
title('V-E Lyapunov envelope');
legend('\mu x^T P x','\eta','Location','best');

%% V-D error measure
figure('Name','V-D error threshold');
plot(t, res_VD.W, 'LineWidth', 1.5); hold on;
yline(rho_D, '--', '\rho_D');
grid on;
xlabel('Time [s]');
ylabel('W_D = e_u^2');
title('V-D fixed-threshold trigger');
legend('e_u^2','\rho_D','Location','best');

%% Trigger values
figure('Name','Trigger values');
plot(t, res_VD.W - rho_D, 'LineWidth', 1.4); hold on;
plot(t, res_VE.V - res_VE.eta, 'LineWidth', 1.4);
plot(t, res_VA.W - max(res_VA.V, res_VA.eta), '--', 'LineWidth', 1.2);
plot(t, res_VC.W - res_VC.V, '-.', 'LineWidth', 1.2);
yline(0,'k:');
grid on;
xlabel('Time [s]');
ylabel('Trigger value');
title('Trigger-condition values');
legend('V-D: W-\rho','V-E: V-\eta','V-A: W-max(V,\eta)','V-C: W-V','Location','best');

%% Transmission instants
figure('Name','Transmission instants');
stem(res_VD.eventTimes, ones(size(res_VD.eventTimes))*1.2, 'filled'); hold on;
stem(res_VE.eventTimes, ones(size(res_VE.eventTimes))*0.9, 'filled');
stem(res_VA.eventTimes, ones(size(res_VA.eventTimes))*0.6, 'filled');
stem(res_VC.eventTimes, ones(size(res_VC.eventTimes))*0.3, 'filled');
grid on;
xlabel('Time [s]');
ylabel('Event marker');
title('Transmission instants');
legend('V-D','V-E','V-A','V-C','Location','best');
yticks([0.3 0.6 0.9 1.2]);
yticklabels({'V-C','V-A','V-E','V-D'});

%% Inter-event intervals
figure('Name','Inter-event intervals');
hold on;
if numel(res_VD.eventTimes) > 1
    plot(diff(res_VD.eventTimes), 'LineWidth', 1.4);
end
if numel(res_VE.eventTimes) > 1
    plot(diff(res_VE.eventTimes), 'LineWidth', 1.4);
end
if numel(res_VA.eventTimes) > 1
    plot(diff(res_VA.eventTimes), '--', 'LineWidth', 1.2);
end
if numel(res_VC.eventTimes) > 1
    plot(diff(res_VC.eventTimes), '-.', 'LineWidth', 1.2);
end
grid on;
xlabel('Event index');
ylabel('Inter-event time [s]');
title('Inter-event intervals');
legend('V-D','V-E','V-A','V-C','Location','best');

%% Update count comparison
figure('Name','Update count comparison');
counts = [res_VD.eventCount, res_VE.eventCount, res_VA.eventCount, res_VC.eventCount, res_Per.eventCount];
bar(counts);
grid on;
set(gca,'XTickLabel',{'V-D','V-E','V-A','V-C','Periodic'});
ylabel('Number of updates');
title('Update count comparison');

%% Optional: save figures as cropped PDFs for LaTeX
% exportgraphics(findobj('Name','Tilt angle comparison'), 'VD_VE_theta_crop.pdf');
% exportgraphics(findobj('Name','Wheel angle comparison'), 'VD_VE_gamma_crop.pdf');
% exportgraphics(findobj('Name','Control input comparison'), 'VD_VE_control_crop.pdf');
% exportgraphics(findobj('Name','V-E Lyapunov envelope'), 'VE_envelope_crop.pdf');
% exportgraphics(findobj('Name','V-D error threshold'), 'VD_threshold_crop.pdf');
% exportgraphics(findobj('Name','Transmission instants'), 'VD_VE_events_crop.pdf');
% exportgraphics(findobj('Name','Inter-event intervals'), 'VD_VE_intervals_crop.pdf');
% exportgraphics(findobj('Name','Update count comparison'), 'VD_VE_counts_crop.pdf');

%% Local simulation function
function res = simulate_ET_robot(A, B, K, P, x0, dt, Tsim, uMax, Tmin, mu, ...
                                 mode, rho_D, lambda_eta_VE, alpha_VE, V_floor_VE, eta0_VE, ...
                                 lambda_eta_VA, eta0_VA, Ts_periodic)

    N = round(Tsim/dt) + 1;
    t = (0:N-1)*dt;

    x = zeros(4,N);
    u = zeros(1,N);
    Vlog = zeros(1,N);
    Wlog = zeros(1,N);
    etalog = zeros(1,N);
    Vdotlog = zeros(1,N);

    x(:,1) = x0(:);

    u_nom = saturate_scalar(-K*x(:,1), uMax);
    u_hold = u_nom;

    eventCount = 1;
    eventTimes = 0;
    timeSinceEvent = 0;
    timeSincePeriodic = 0;

    if mode == "VA"
        eta = eta0_VA;
    elseif mode == "VE"
        eta = eta0_VE;
    else
        eta = 0;
    end

    for k = 1:N-1
        xk = x(:,k);

        u_nom = saturate_scalar(-K*xk, uMax);
        e_u = u_hold - u_nom;

        V = mu*(xk.'*P*xk);
        W = e_u^2;

        xdot_hold = A*xk + B*u_hold;
        Vdot = 2*mu*(xk.'*P*xdot_hold);

        trigger = false;

        switch mode

            case "VD"
                if timeSinceEvent >= Tmin
                    trigger = (W >= rho_D);
                end

                if trigger
                    u_hold = u_nom;
                    eventCount = eventCount + 1;
                    eventTimes(end+1) = t(k); %#ok<AGROW>
                    timeSinceEvent = 0;
                end

            case "VE"
                eta = eta - dt*lambda_eta_VE*eta;
                if eta < 0
                    eta = 0;
                end

                envelope_hit = (V >= eta);
                not_decreasing_enough = (Vdot >= -alpha_VE*V);
                away_from_origin = (V >= V_floor_VE);

                if timeSinceEvent >= Tmin
                    trigger = envelope_hit && not_decreasing_enough && away_from_origin;
                end

                if trigger
                    u_hold = u_nom;
                    eta = V;          % eta^+ = V(x)
                    eventCount = eventCount + 1;
                    eventTimes(end+1) = t(k); %#ok<AGROW>
                    timeSinceEvent = 0;
                end

            case "VA"
                eta = eta - dt*lambda_eta_VA*eta;
                if eta < 0
                    eta = 0;
                end

                if timeSinceEvent >= Tmin
                    trigger = (W >= max(V, eta));
                end

                if trigger
                    u_hold = u_nom;
                    eta = W;
                    eventCount = eventCount + 1;
                    eventTimes(end+1) = t(k); %#ok<AGROW>
                    timeSinceEvent = 0;
                end

            case "VC"
                if timeSinceEvent >= Tmin
                    trigger = (W >= V);
                end

                if trigger
                    u_hold = u_nom;
                    eventCount = eventCount + 1;
                    eventTimes(end+1) = t(k); %#ok<AGROW>
                    timeSinceEvent = 0;
                end

            case "Periodic"
                if timeSincePeriodic >= Ts_periodic
                    u_hold = u_nom;
                    eventCount = eventCount + 1;
                    timeSincePeriodic = 0;
                end

            otherwise
                error('Unknown mode. Use "VD", "VE", "VA", "VC", or "Periodic".');
        end

        u(k) = u_hold;
        Vlog(k) = V;
        Wlog(k) = W;
        etalog(k) = eta;
        Vdotlog(k) = Vdot;

        xdot = A*xk + B*u_hold;
        x(:,k+1) = xk + dt*xdot;

        timeSinceEvent = timeSinceEvent + dt;
        timeSincePeriodic = timeSincePeriodic + dt;
    end

    u(end) = u(end-1);
    Vlog(end) = Vlog(end-1);
    Wlog(end) = Wlog(end-1);
    etalog(end) = etalog(end-1);
    Vdotlog(end) = Vdotlog(end-1);

    res.t = t;
    res.x = x;
    res.u = u;
    res.V = Vlog;
    res.W = Wlog;
    res.eta = etalog;
    res.Vdot = Vdotlog;
    res.eventCount = eventCount;
    res.eventTimes = eventTimes;
end

function y = saturate_scalar(y, ymax)
    if y > ymax
        y = ymax;
    elseif y < -ymax
        y = -ymax;
    end
end
