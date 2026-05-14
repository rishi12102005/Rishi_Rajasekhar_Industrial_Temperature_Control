clc;
clear;
close all;

%% ============================================================
% INDUSTRIAL FURNACE TEMPERATURE CONTROL
% HYBRID PID + DISTURBANCE REJECTION + FILTERING
%% ============================================================

disp('==============================================')
disp('ADVANCED INDUSTRIAL FURNACE CONTROL SYSTEM')
disp('==============================================')

%% ============================================================
% STEP 1 : DEFINE SYSTEM
%% ============================================================

s = tf('s');

% Furnace transfer function: G(s) = 2 / (10s + 1)
G = 2 / (10*s + 1);

disp(' ')
disp('Plant Transfer Function:')
G

%% ============================================================
% STEP 2 : DESIGN PID CONTROLLER
%% ============================================================

Kp = 2.5;
Ki = 0.35;
Kd = 0.8;

% BUG FIX 1 ── pid() without a derivative filter causes infinite
% high-frequency gain and a pure differentiator.  Always supply
% the filter coefficient N (or Tf = 1/N).  N = 10 is standard.
C = pid(Kp, Ki, Kd, 1/10);   % Tf = 0.1 s

disp(' ')
disp('PID Controller:')
C

%% ============================================================
% STEP 3 : CLOSED LOOP SYSTEM
%% ============================================================

CL = feedback(C * G, 1);

% Disturbance-to-output transfer function
% d(t) enters at the plant input, so:
%   G_d(s) = G(s) / (1 + C(s)*G(s))
% BUG FIX 2 ── previously disturbance was simply added to y as
% a constant offset which (a) ignored the controller fighting
% back and (b) caused a row/column dimension mismatch crash.
G_dist = feedback(G, C);      % dist -> output (controller rejects it)

disp(' ')
disp('Closed Loop System:')
CL

%% ============================================================
% STEP 4 : SIMULATION TIME
%% ============================================================

t = (0:0.01:50)';             % column vector — consistent with lsim/step

%% ============================================================
% STEP 5 : NORMAL STEP RESPONSE
%% ============================================================

y = step(CL, t);              % y is a column vector matching t

%% ============================================================
% STEP 6 : DISTURBANCE MODEL  (CORRECTED)
%% ============================================================

% BUG FIX 2 (continued) ── use lsim() through G_dist so the
% closed-loop controller actively rejects the heat loss.
% The disturbance input is a step of magnitude -0.25 from t = 15 s.

disturbance        = zeros(size(t));       % column vector
disturbance(t >= 15) = -0.25;

% Disturbance contribution to the output (controller fights back)
y_dist = lsim(G_dist, disturbance, t);    % column vector

% Total disturbed output = nominal response + disturbance response
y_disturbed = y + y_dist;                 % both column vectors: no crash

%% ============================================================
% STEP 7 : SENSOR FILTER
%% ============================================================

% Low-pass filter: H(s) = 1 / (0.5s + 1)
tau_filter = 0.5;
H = 1 / (tau_filter*s + 1);

% Filter the disturbed output
filtered_output = lsim(H, y_disturbed, t);

%% ============================================================
% STEP 8 : PERFORMANCE ANALYSIS
%% ============================================================

info = stepinfo(CL);

disp(' ')
disp('==============================================')
disp('PERFORMANCE PARAMETERS')
disp('==============================================')
disp(info)

%% ============================================================
% STEP 9 : CONTROL SIGNAL  (CORRECTED)
%% ============================================================

% BUG FIX 3 ── error must be computed against the *disturbed*
% filtered output, not against the open-loop y, so that the
% control effort shown is the true closed-loop signal including
% the controller's disturbance rejection effort.

error_signal     = 1 - filtered_output;           % column vector

integral_term    = cumtrapz(t, error_signal);      % column vector

% BUG FIX 3b ── diff returns a vector of length N-1; prepend 0
% and divide by dt to get a proper numerical derivative.
dt               = t(2) - t(1);
derivative_term  = [0; diff(error_signal)] / dt;  % column vector

u = Kp * error_signal + ...
    Ki * integral_term + ...
    Kd * derivative_term;

%% ============================================================
% STEP 10 : ENERGY ANALYSIS
%% ============================================================

energy = trapz(t, abs(u));

disp(' ')
disp('==============================================')
disp('ENERGY ANALYSIS')
disp('==============================================')
fprintf('Estimated Energy Consumption = %.2f Units\n', energy);

%% ============================================================
% STEP 11 : DISTURBANCE OBSERVER  (CORRECTED)
%% ============================================================

% BUG FIX 4 ── the original estimate (y_disturbed - filtered_output)
% only captured the filter's lag, not the actual disturbance.
% Correct estimate: invert the plant steady-state to recover d̂(t).
%
%   d̂(t) = (y_disturbed - y) / dcgain(G_dist)
%
% This gives the best linear estimate of d(t) from output residuals.

dc_Gd             = dcgain(G_dist);
estimated_disturbance = (y_disturbed - y) / dc_Gd;

%% ============================================================
% STEP 12 : PLOTS
%% ============================================================

%% Normal step response
figure;
plot(t, y, 'b', 'LineWidth', 2)
grid on
title('Closed Loop Step Response')
xlabel('Time (seconds)')
ylabel('Temperature')
yline(1,    'k--', 'Setpoint',  'LabelHorizontalAlignment','left')
yline(1.05, 'r:',  '+5% band',  'LabelHorizontalAlignment','left')
yline(0.95, 'r:',  '-5% band',  'LabelHorizontalAlignment','left')

%% Disturbance response
figure;
plot(t, y_disturbed, 'r', 'LineWidth', 2)
hold on
plot(t, y,           'b--', 'LineWidth', 1.5)
grid on
title('Disturbance Rejection (heat loss at t = 15 s)')
xlabel('Time (seconds)')
ylabel('Temperature')
legend('Disturbed output', 'Nominal response', 'Location', 'southeast')
xline(15, 'k--', 'Disturbance onset', 'LabelVerticalAlignment','bottom')

%% Filtered response
figure;
plot(t, filtered_output, 'm', 'LineWidth', 2)
hold on
plot(t, y_disturbed, 'r--', 'LineWidth', 1.2)
grid on
title('Filtered Temperature Response')
xlabel('Time (seconds)')
ylabel('Temperature')
legend('Filtered output', 'Disturbed output', 'Location', 'southeast')

%% Control effort
figure;
plot(t, u, 'Color', [0.06 0.43 0.34], 'LineWidth', 2)
grid on
title('PID Control Effort (heater power)')
xlabel('Time (seconds)')
ylabel('Heater Power')
yline(0, 'k--')

%% Disturbance estimate
figure;
plot(t, disturbance,             'r-',  'LineWidth', 2); hold on
plot(t, estimated_disturbance,   'b--', 'LineWidth', 2)
grid on
title('Disturbance Observer — Actual vs Estimated')
xlabel('Time (seconds)')
ylabel('Disturbance magnitude')
legend('Actual d(t)', 'Estimated d̂(t)', 'Location', 'southwest')

%% ============================================================
% STEP 13 : COMPARISON PLOT
%% ============================================================

figure;
plot(t, y,               'b',  'LineWidth', 2); hold on
plot(t, y_disturbed,     'r',  'LineWidth', 2)
plot(t, filtered_output, 'm',  'LineWidth', 2)
grid on
title('System Comparison')
xlabel('Time (seconds)')
ylabel('Temperature')
legend('Normal Response', 'Disturbed Response', 'Filtered Response', ...
       'Location', 'southeast')
xline(15, 'k--', 'Disturbance onset', 'LabelVerticalAlignment','bottom')

%% ============================================================
% STEP 14 : DISPLAY FINAL RESULTS
%% ============================================================

ss_error = abs(1 - dcgain(CL));

disp(' ')
disp('==============================================')
disp('FINAL RESULTS')
disp('==============================================')
fprintf('Rise Time       = %.2f seconds\n',  info.RiseTime);
fprintf('Settling Time   = %.2f seconds\n',  info.SettlingTime);
fprintf('Overshoot       = %.2f %%\n',       info.Overshoot);
fprintf('Steady State    = %.4f\n',          dcgain(CL));
fprintf('SS Error        = %.4f\n',          ss_error);
fprintf('Energy (|u| dt) = %.2f Units\n',    energy);

disp(' ')
disp('--- Spec Check ---')
if info.Overshoot < 5
    disp('Overshoot   < 5 %  : PASS')
else
    disp('Overshoot   < 5 %  : FAIL')
end
if info.SettlingTime < 50
    disp('Settling    < 50 s : PASS')
else
    disp('Settling    < 50 s : FAIL')
end
if ss_error < 0.005
    disp('SS error    < 0.005: PASS')
else
    disp('SS error    < 0.005: FAIL')
end

disp(' ')
disp('System Successfully Simulated')
disp(' ')