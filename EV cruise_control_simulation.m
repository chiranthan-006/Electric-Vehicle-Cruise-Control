%%  EV CRUISE CONTROL — FULL SIMULATION
%%
%%  Features:
%%    - Interactive cruise speed input
%%    - First-order plant with PID + derivative filter
%%    - Uphill / downhill disturbances and brake event
%%    - 6-panel analysis figure + zoomed disturbance figure
%%    - Quantitative performance report (SS error, overshoot,
%%      settling time, disturbance recovery)
%%
%%  Usage:  Run  cruise_control_simulation  in MATLAB command window.
%%
%%  Author:  [Your Name]   |   Version: 2.0
%% ============================================================

clear; close all; clc;

%% ─────────────────────────────────────────────────────────────
%%  SECTION 0  :  USER INPUT
%% ─────────────────────────────────────────────────────────────
fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║  EV CRUISE CONTROL SYSTEM  —  SIMULATION SETUP   ║\n');
fprintf('╚══════════════════════════════════════════════════╝\n\n');

SPEED_MIN_KMH = 10;
SPEED_MAX_KMH = 200;
DEFAULT_SPEED  = 60;

cruise_speed_kmh = input(sprintf('  Enter desired cruise speed (%d-%d km/h): ', ...
                                  SPEED_MIN_KMH, SPEED_MAX_KMH));

if isempty(cruise_speed_kmh) || ...
   ~isnumeric(cruise_speed_kmh) || ...
   cruise_speed_kmh < SPEED_MIN_KMH || ...
   cruise_speed_kmh > SPEED_MAX_KMH

    cruise_speed_kmh = DEFAULT_SPEED;
    fprintf('  WARNING: Invalid input — using default: %d km/h\n', DEFAULT_SPEED);
end

fprintf('\n  Cruise speed set: %.0f km/h\n', cruise_speed_kmh);
fprintf('  Running simulation...\n\n');

%% ─────────────────────────────────────────────────────────────
%%  SECTION 1  :  SYSTEM PARAMETERS
%% ─────────────────────────────────────────────────────────────

% ── PID Gains ────────────────────────────────────────────────
% Tuned for plant  G(s) = 1 / (tau*s + 1),  tau = 5 s
% Closed-loop bandwidth ~1.8 rad/s;  phase margin ~55 deg
Kp = 8.5;     % Proportional gain
Ki = 2.0;     % Integral gain        [eliminates steady-state error]
Kd = 5.0;     % Derivative gain      [improves transient response]
Tf = 0.1;     % Derivative filter TC [reduces noise sensitivity; N = 1/Tf = 10]

% ── Plant Model ───────────────────────────────────────────────
% First-order EV drive model:  tau * dy/dt + y = u + d
%   y = normalised speed  [0..1]
%   u = throttle command  [0..1]
%   d = road-slope disturbance [normalised]
tau = 5;      % Time constant [s]

% ── Normalisation ─────────────────────────────────────────────
% throttle = 1  =>  steady-state normalised speed = 1  =>  cruise_speed_kmh
speed_scale = cruise_speed_kmh;   % [km/h per normalised unit]
ref_norm    = 1.0;                 % Normalised reference (= full cruise speed)

% ── Simulation Grid ───────────────────────────────────────────
dt      = 0.01;          % Time step [s]  (100 Hz — well above plant bandwidth)
T_total = 40;            % Total simulation duration [s]
t       = 0 : dt : T_total;
N       = length(t);

%% ─────────────────────────────────────────────────────────────
%%  SECTION 2  :  EVENT TIMELINE
%% ─────────────────────────────────────────────────────────────
% All times in seconds from simulation start

t_cruise_on    = 4;    % Driver presses cruise button
t_uphill       = 12;   % Road tilts uphill
t_uphill_end   = 22;   % Road levels out
t_downhill     = 24;   % Road tilts downhill
t_downhill_end = 34;   % Road levels out
t_brake        = 37;   % Driver brakes — cruise disengages

% Normalised disturbance magnitudes
%   Negative  =>  load increase (uphill: vehicle tends to slow down)
%   Positive  =>  load decrease (downhill: vehicle tends to speed up)
DIST_UPHILL   = -0.18;
DIST_DOWNHILL = +0.15;

%% ─────────────────────────────────────────────────────────────
%%  SECTION 3  :  PRE-ALLOCATE STATE ARRAYS
%% ─────────────────────────────────────────────────────────────
speed_norm   = zeros(1, N);   % Normalised vehicle speed       [0..1]
speed_kmh    = zeros(1, N);   % Vehicle speed                  [km/h]
setpoint_kmh = zeros(1, N);   % Active cruise setpoint         [km/h]
error_kmh    = zeros(1, N);   % Speed tracking error           [km/h]
throttle_cmd = zeros(1, N);   % Throttle command (PID output)  [0..1]
disturbance  = zeros(1, N);   % Road-slope disturbance         [normalised]
cruise_on    = false(1, N);   % Cruise-active flag             [logical]

% Manual-drive initial throttle: brings vehicle to ~40% of cruise speed
% before the cruise button is pressed, giving a realistic approach phase.
THROTTLE_INIT = 0.40;

%% ─────────────────────────────────────────────────────────────
%%  SECTION 4  :  PID INTEGRATOR STATES  (all initialised to 0)
%% ─────────────────────────────────────────────────────────────
pid_integral   = 0;
pid_prev_err   = 0;
pid_prev_deriv = 0;
cruise_active  = false;
setpoint_norm  = 0;

%% ─────────────────────────────────────────────────────────────
%%  SECTION 5  :  MAIN SIMULATION LOOP
%%
%%  Euler forward integration. Each iteration i computes:
%%    (a) current disturbance value
%%    (b) cruise / brake state machine
%%    (c) PID or manual throttle
%%    (d) plant state update  (speed_norm)
%%    (e) record all signals for post-processing
%% ─────────────────────────────────────────────────────────────

for i = 2 : N

    tc = t(i-1);   % Simulation time at the start of this step [s]

    %% ── 5a.  Road-slope disturbance profile ─────────────────────────────
    d = 0;
    if tc >= t_uphill   && tc < t_uphill_end,   d = DIST_UPHILL;   end
    if tc >= t_downhill && tc < t_downhill_end, d = DIST_DOWNHILL; end
    disturbance(i-1) = d;

    %% ── 5b.  Cruise / brake state machine ───────────────────────────────
    % Priority rule: brake overrides cruise engagement.
    if tc >= t_brake
        cruise_active = false;
        pid_integral  = 0;   % Anti-windup: clear integrator on disengage
    elseif tc >= t_cruise_on && ~cruise_active
        % Engage cruise: lock setpoint to the normalised reference speed
        cruise_active = true;
        setpoint_norm = ref_norm;
    end

    %% ── 5c.  Compute throttle command ───────────────────────────────────
    if cruise_active
        % ── PID control ──────────────────────────────────────────────────
        err          = setpoint_norm - speed_norm(i-1);
        pid_integral = pid_integral + err * dt;

        % Derivative with first-order low-pass filter:
        %   Continuous:  D(s) = Kd * s / (Tf*s + 1)
        %   Discrete:    alpha = Tf/dt
        %                filt_deriv = (raw_deriv + alpha*prev_deriv) / (1 + alpha)
        raw_deriv  = (err - pid_prev_err) / dt;
        alpha      = Tf / dt;
        filt_deriv = (raw_deriv + alpha * pid_prev_deriv) / (1 + alpha);

        u = Kp * err + Ki * pid_integral + Kd * filt_deriv;
        u = max(0, min(1, u));   % Actuator saturation clamp [0, 1]

        pid_prev_err   = err;
        pid_prev_deriv = filt_deriv;

    else
        % ── Manual / coast throttle ───────────────────────────────────────
        if tc < t_cruise_on
            % Linear ramp-up: reach THROTTLE_INIT after 2 s of driving
            u = min(THROTTLE_INIT, THROTTLE_INIT * tc / 2);
        else
            % Post-brake coast: zero throttle — vehicle decelerates naturally
            u = 0;
        end
        % Reset PID states to prevent integrator windup on re-engagement
        pid_integral   = 0;
        pid_prev_err   = 0;
        pid_prev_deriv = 0;
    end

    throttle_cmd(i-1) = u;

    %% ── 5d.  Plant dynamics  (Euler forward integration) ────────────────
    %  First-order ODE:  tau * dy/dt = -y + u + d
    %  => dy/dt = (-speed_norm + throttle + disturbance) / tau
    dydt          = (-speed_norm(i-1) + u + disturbance(i-1)) / tau;
    speed_norm(i) = speed_norm(i-1) + dydt * dt;
    speed_norm(i) = max(0, speed_norm(i));   % Physical constraint: v >= 0

    %% ── 5e.  Record signals ─────────────────────────────────────────────
    speed_kmh(i-1)    = speed_norm(i-1) * speed_scale;
    setpoint_kmh(i-1) = setpoint_norm   * speed_scale;
    error_kmh(i-1)    = (setpoint_norm - speed_norm(i-1)) * speed_scale;
    cruise_on(i-1)    = cruise_active;

end  % end simulation loop

%% Fill the final sample (loop writes i-1, so index N is never written inside)
speed_kmh(end)    = speed_norm(end) * speed_scale;
setpoint_kmh(end) = setpoint_norm   * speed_scale;
error_kmh(end)    = error_kmh(end-1);
throttle_cmd(end) = throttle_cmd(end-1);
cruise_on(end)    = cruise_on(end-1);
disturbance(end)  = disturbance(end-1);

%% ─────────────────────────────────────────────────────────────
%%  SECTION 6  :  PERFORMANCE METRICS
%% ─────────────────────────────────────────────────────────────
BAND_PCT = 2;                                   % Tolerance band [%]
band_kmh = BAND_PCT / 100 * cruise_speed_kmh;  % Tolerance band [km/h]

% ── 6a.  Steady-state error (settled region, before first disturbance) ────
% Window: 8 s after cruise engagement up to start of uphill slope
ss_mask           = (t >= t_cruise_on + 8) & (t < t_uphill);
ss_error_mean_kmh = NaN;
ss_error_pct      = NaN;
if any(ss_mask)
    ss_error_mean_kmh = mean(abs(error_kmh(ss_mask)));
    ss_error_pct      = ss_error_mean_kmh / cruise_speed_kmh * 100;
end

% ── 6b.  Overshoot (peak speed above setpoint during cruise phase) ────────
cruise_mask   = cruise_on & (t < t_brake);
overshoot_pct = NaN;
if any(cruise_mask)
    max_speed_kmh = max(speed_kmh(cruise_mask));
    overshoot_pct = max(0, (max_speed_kmh - cruise_speed_kmh) / cruise_speed_kmh * 100);
end

% ── 6c.  Settling time (first time error enters +/-BAND_PCT band) ─────────
%  Requires MIN_CONSEC consecutive samples inside band for robust detection.
settling_time_s = NaN;
cruise_start_i  = find(cruise_on, 1, 'first');
MIN_CONSEC = 10;   % 0.1 s at dt=0.01 s — avoids false triggers on ringing
if ~isempty(cruise_start_i)
    in_band   = abs(error_kmh(cruise_start_i:end)) <= band_kmh;
    settled_i = strfind(double(in_band), ones(1, MIN_CONSEC));
    if ~isempty(settled_i)
        abs_idx         = cruise_start_i + settled_i(1) - 1;
        settling_time_s = t(abs_idx) - t_cruise_on;
    end
end

% ── 6d.  Disturbance recovery metrics ────────────────────────────────────
[max_dev_up_kmh, recovery_time_up_s] = disturbanceMetrics( ...
    t, error_kmh, t_uphill, t_uphill_end, band_kmh);

[max_dev_dn_kmh, recovery_time_dn_s] = disturbanceMetrics( ...
    t, error_kmh, t_downhill, t_downhill_end, band_kmh);

%% ─────────────────────────────────────────────────────────────
%%  SECTION 7  :  PERFORMANCE REPORT  (command window)
%% ─────────────────────────────────────────────────────────────
SEPL = '╠══════════════════════════════════════════════════╣';
fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║              PERFORMANCE REPORT                  ║\n');
fprintf('%s\n', SEPL);
fprintf('║  Cruise setpoint   : %5.0f km/h                 ║\n', cruise_speed_kmh);
fprintf('║  Simulation time   : %5.0f s                    ║\n', T_total);
fprintf('║  Time step  dt     : %5.3f s  (%.0f Hz)          ║\n', dt, 1/dt);
fprintf('║  Tolerance band    : +/-%d%%  (+/-%.2f km/h)      ║\n', BAND_PCT, band_kmh);
fprintf('%s\n', SEPL);
fprintf('║  STEADY-STATE (settled phase, before disturbance)║\n');
printMetricLine('Mean |error|', ss_error_mean_kmh, 'km/h', ss_error_pct, '%', 2, 'PASS < 2%');
fprintf('%s\n', SEPL);
fprintf('║  TRANSIENT RESPONSE                              ║\n');
printMetricLine('Overshoot',    overshoot_pct,      '%',    [],           '',  5, 'PASS < 5%');
printValueLine ('Settling time (2%% band)', settling_time_s, 's');
fprintf('%s\n', SEPL);
fprintf('║  DISTURBANCE REJECTION                           ║\n');
printValueLine ('Uphill  max deviation',    max_dev_up_kmh,    'km/h');
printValueLine ('Uphill  recovery time',    recovery_time_up_s,'s');
printValueLine ('Downhill max deviation',   max_dev_dn_kmh,    'km/h');
printValueLine ('Downhill recovery time',   recovery_time_dn_s,'s');
fprintf('╚══════════════════════════════════════════════════╝\n\n');

%% ─────────────────────────────────────────────────────────────
%%  SECTION 8  :  PLOTS
%% ─────────────────────────────────────────────────────────────

% ── Colour palette ───────────────────────────────────────────
CLR.ref      = [0.85 0.15 0.15];   % Red    – setpoint
CLR.speed    = [0.10 0.45 0.85];   % Blue   – actual speed
CLR.throttle = [0.15 0.70 0.35];   % Green  – throttle
CLR.error    = [0.90 0.50 0.10];   % Orange – error
CLR.dist     = [0.55 0.10 0.75];   % Purple – disturbance
CLR.band     = [0.50 0.50 0.50];   % Grey   – tolerance band lines

% ── Event-label helper (anonymous function) ──────────────────
% Usage: markEvent(ax, time, label, colour)
markEvent = @(ax, x, lbl, clr) xline(ax, x, '--', lbl, ...
    'Color', clr, ...
    'LabelVerticalAlignment',   'top', ...
    'LabelHorizontalAlignment', 'center', ...
    'FontSize', 7.5, 'LineWidth', 1.2);

% ── Figure 1 : Full Analysis (3 rows x 2 cols) ───────────────
fig1 = figure('Name',     'EV Cruise Control - Full Analysis', ...
              'Position', [40 40 1400 900], ...
              'Color',    [1 1 1]);

%% Plot 1 : Speed Tracking  (spans full top row) ──────────────────────────
ax1   = subplot(3, 2, [1 2]);
y_max = max(speed_kmh) * 1.15;

shadePeriod(ax1, t_uphill,   t_uphill_end,   y_max, [1.00 0.92 0.92]);
shadePeriod(ax1, t_downhill, t_downhill_end, y_max, [0.90 0.95 1.00]);
shadePeriod(ax1, t_brake,    T_total,        y_max, [1.00 0.97 0.90]);

hold(ax1, 'on');
plot(ax1, t, setpoint_kmh, '--', 'Color', CLR.ref,   'LineWidth', 2.0, 'DisplayName', 'Setpoint');
plot(ax1, t, speed_kmh,    '-',  'Color', CLR.speed,  'LineWidth', 2.5, 'DisplayName', 'Actual speed');

yline(ax1, cruise_speed_kmh * 1.02, ':', 'Color', CLR.band, 'LineWidth', 1);
yline(ax1, cruise_speed_kmh * 0.98, ':', 'Color', CLR.band, 'LineWidth', 1, ...
      'Label', sprintf('+/-%d%% band', BAND_PCT));

markEvent(ax1, t_cruise_on,    'Cruise ON',   [0.10 0.65 0.10]);
markEvent(ax1, t_uphill,       'Uphill',      [0.80 0.10 0.10]);
markEvent(ax1, t_uphill_end,   'Flat',         CLR.band);
markEvent(ax1, t_downhill,     'Downhill',    [0.10 0.40 0.80]);
markEvent(ax1, t_downhill_end, 'Flat',         CLR.band);
markEvent(ax1, t_brake,        'Brake',       [0.85 0.30 0.00]);

formatAxes(ax1, 'Time (s)', 'Speed (km/h)', ...
    sprintf('Speed Tracking  |  Cruise target: %.0f km/h', cruise_speed_kmh), 13);
legend(ax1, 'Setpoint', 'Actual speed', sprintf('+/-%d%% limit', BAND_PCT), ...
       'Location', 'southwest', 'FontSize', 9);
xlim(ax1, [0 T_total]);
ylim(ax1, [0 y_max]);

%% Plot 2 : Tracking Error ────────────────────────────────────────────────
ax2 = subplot(3, 2, 3);
hold(ax2, 'on');
yline(ax2,  band_kmh, '--', 'Color', [0.15 0.65 0.15], 'LineWidth', 1.2, ...
      'Label', sprintf('+%d%%', BAND_PCT));
yline(ax2, -band_kmh, '--', 'Color', [0.15 0.65 0.15], 'LineWidth', 1.2, ...
      'Label', sprintf('-%d%%', BAND_PCT));
yline(ax2,  0,        '-',  'Color', [0.3 0.3 0.3],    'LineWidth', 0.8);
plot(ax2, t, error_kmh, '-', 'Color', CLR.error, 'LineWidth', 2);

markEvent(ax2, t_cruise_on, 'Cruise ON', [0.10 0.65 0.10]);
markEvent(ax2, t_uphill,    'Uphill',    [0.80 0.10 0.10]);
markEvent(ax2, t_downhill,  'Downhill',  [0.10 0.40 0.80]);
markEvent(ax2, t_brake,     'Brake',     [0.85 0.30 0.00]);

formatAxes(ax2, 'Time (s)', 'Error (km/h)', 'Tracking Error', 11);
xlim(ax2, [0 T_total]);

%% Plot 3 : Throttle Command ──────────────────────────────────────────────
ax3 = subplot(3, 2, 4);
hold(ax3, 'on');
plot(ax3, t, throttle_cmd,       '-',  'Color', CLR.throttle, 'LineWidth', 2.0, ...
     'DisplayName', 'Throttle');
plot(ax3, t, double(cruise_on) * 0.08, ':', 'Color', [0.7 0.1 0.7], 'LineWidth', 1.5, ...
     'DisplayName', 'Cruise active (scaled x0.08)');

yline(ax3, 0, 'k--', 'LineWidth', 0.8);
yline(ax3, 1, 'k--', 'LineWidth', 0.8);

markEvent(ax3, t_cruise_on, 'Cruise ON', [0.10 0.65 0.10]);
markEvent(ax3, t_uphill,    'Uphill',    [0.80 0.10 0.10]);
markEvent(ax3, t_downhill,  'Downhill',  [0.10 0.40 0.80]);
markEvent(ax3, t_brake,     'Brake',     [0.85 0.30 0.00]);

formatAxes(ax3, 'Time (s)', 'Throttle [0-1]', 'PID Throttle Command', 11);
legend(ax3, 'Location', 'best', 'FontSize', 8);
xlim(ax3, [0 T_total]);
ylim(ax3, [-0.05 1.10]);

%% Plot 4 : Road Disturbance ──────────────────────────────────────────────
ax4 = subplot(3, 2, 5);
area(ax4, t, disturbance, ...
     'FaceColor', CLR.dist, 'FaceAlpha', 0.4, ...
     'EdgeColor', CLR.dist, 'LineWidth', 1.5);
hold(ax4, 'on');
yline(ax4, 0, 'k-', 'LineWidth', 0.8);

markEvent(ax4, t_uphill,       'Uphill',   [0.80 0.10 0.10]);
markEvent(ax4, t_uphill_end,   'Flat',      CLR.band);
markEvent(ax4, t_downhill,     'Downhill', [0.10 0.40 0.80]);
markEvent(ax4, t_downhill_end, 'Flat',      CLR.band);

formatAxes(ax4, 'Time (s)', 'Disturbance [norm.]', 'Road-Slope Disturbance', 11);
xlim(ax4, [0 T_total]);

%% Plot 5 : Performance Summary Bar Chart ─────────────────────────────────
ax5 = subplot(3, 2, 6);
hold(ax5, 'on');

metric_labels = {'SS Error (%)', 'Overshoot (%)'};
actual_vals   = [ss_error_pct,   overshoot_pct];
limit_vals    = [2,              5];
bar_clrs_act  = [CLR.error; CLR.speed];

for k = 1 : 2
    val = actual_vals(k);
    lim = limit_vals(k);

    bar(ax5, k - 0.20, val, 0.35, 'FaceColor', bar_clrs_act(k,:), 'EdgeColor', 'none');
    bar(ax5, k + 0.20, lim, 0.35, 'FaceColor', [0.75 0.75 0.75],  'EdgeColor', 'none');

    if ~isnan(val)
        if val <= lim
            label_txt = 'PASS';   label_clr = [0.10 0.60 0.10];
        else
            label_txt = 'FAIL';   label_clr = [0.80 0.10 0.10];
        end
        text(ax5, k, max(val, lim) + 0.15, label_txt, ...
             'HorizontalAlignment', 'center', ...
             'Color', label_clr, 'FontWeight', 'bold', 'FontSize', 9);
    end
end

set(ax5, 'XTick', 1:2, 'XTickLabel', metric_labels, 'FontSize', 10);
legend(ax5, 'Actual', 'Limit', 'Location', 'northwest', 'FontSize', 9);
ylabel(ax5, 'Percentage (%)');
title(ax5, 'Performance vs. Requirements', 'FontSize', 11, 'FontWeight', 'bold');
grid(ax5, 'on');
valid_vals = actual_vals(~isnan(actual_vals));
ylim(ax5, [0, max([valid_vals, limit_vals]) * 1.5 + 0.5]);

%% ── Global figure title ──────────────────────────────────────
sgtitle(fig1, sprintf( ...
    'EV Cruise Control  |  Target: %d km/h  |  PID  Kp=%.1f  Ki=%.1f  Kd=%.1f  Tf=%.2f s', ...
    cruise_speed_kmh, Kp, Ki, Kd, Tf), ...
    'FontSize', 13, 'FontWeight', 'bold');

%% ─────────────────────────────────────────────────────────────
%%  SECTION 9  :  ZOOMED DISTURBANCE FIGURES
%% ─────────────────────────────────────────────────────────────
fig2 = figure('Name',     'Disturbance Rejection - Zoomed', ...
              'Position', [60 60 1200 480], ...
              'Color',    [1 1 1]);

ZOOM_MARGIN = 2;   % Seconds of context to show either side of the event

%% ── Zoom: Uphill ─────────────────────────────────────────────
ax_up = subplot(1, 2, 1);
plotZoomPanel(ax_up, t, speed_kmh, setpoint_kmh, ...
    t_uphill, t_uphill_end, ZOOM_MARGIN, ...
    [1.00 0.92 0.92], CLR, markEvent, ...
    {'Uphill starts', 'Flat again'}, ...
    [0.8 0.1 0.1; 0.5 0.5 0.5], ...
    'Uphill Response (Zoomed)');

%% ── Zoom: Downhill ───────────────────────────────────────────
ax_dn = subplot(1, 2, 2);
plotZoomPanel(ax_dn, t, speed_kmh, setpoint_kmh, ...
    t_downhill, t_downhill_end, ZOOM_MARGIN, ...
    [0.90 0.95 1.00], CLR, markEvent, ...
    {'Downhill starts', 'Flat again'}, ...
    [0.1 0.4 0.8; 0.5 0.5 0.5], ...
    'Downhill Response (Zoomed)');

sgtitle(fig2, 'Disturbance Rejection — Uphill & Downhill', ...
        'FontSize', 13, 'FontWeight', 'bold');

%% ─────────────────────────────────────────────────────────────
%%  SECTION 10  :  SAVE OUTPUTS
%% ─────────────────────────────────────────────────────────────
fname_fig1 = sprintf('cruise_analysis_%dkmh.png',  cruise_speed_kmh);
fname_fig2 = sprintf('disturbance_zoom_%dkmh.png', cruise_speed_kmh);
fname_mat  = sprintf('simulation_data_%dkmh.mat',  cruise_speed_kmh);

saveas(fig1, fname_fig1);
saveas(fig2, fname_fig2);
save(fname_mat, 't', 'speed_kmh', 'setpoint_kmh', ...
     'error_kmh', 'throttle_cmd', 'disturbance', 'cruise_on');

fprintf('Saved:\n  %s\n  %s\n  %s\n\nDone.\n\n', fname_fig1, fname_fig2, fname_mat);


%% =============================================================
%%  LOCAL HELPER FUNCTIONS
%% =============================================================

function [max_dev, recovery_time] = disturbanceMetrics(t, err_kmh, t_start, t_end, band)
%DISTURBANCEMETRICS  Peak deviation and recovery time during a disturbance window.
%
%   [max_dev, recovery_time] = disturbanceMetrics(t, err_kmh, t_start, t_end, band)
%
%   Inputs:
%     t             - time vector [s]
%     err_kmh       - tracking error vector [km/h]
%     t_start       - disturbance start time [s]
%     t_end         - disturbance end   time [s]
%     band          - tolerance band [km/h]
%
%   Outputs:
%     max_dev       - peak absolute error inside the window [km/h]
%     recovery_time - seconds from t_start until |error| <= band [s]
%                     NaN if error never returns to band within window

    i_start = find(t >= t_start, 1, 'first');
    i_end   = find(t >= t_end,   1, 'first');

    if isempty(i_start) || isempty(i_end) || i_start >= i_end
        max_dev       = NaN;
        recovery_time = NaN;
        return;
    end

    window  = err_kmh(i_start : i_end);
    max_dev = max(abs(window));

    rec_i = find(abs(window) <= band, 1, 'last');
    if ~isempty(rec_i)
        recovery_time = t(i_start + rec_i - 1) - t_start;
    else
        recovery_time = NaN;
    end
end

% ─────────────────────────────────────────────────────────────

function shadePeriod(ax, t_lo, t_hi, y_max, face_clr)
%SHADEPERIOD  Draw a shaded background band between two time bounds.
    fill(ax, [t_lo t_hi t_hi t_lo], [0 0 y_max y_max], face_clr, ...
         'EdgeColor', 'none', 'FaceAlpha', 0.5);
    hold(ax, 'on');
end

% ─────────────────────────────────────────────────────────────

function formatAxes(ax, xlab, ylab, ttl, title_fontsize)
%FORMATAXES  Apply consistent grid, labels, and title to an axes object.
    grid(ax, 'on');
    grid(ax, 'minor');
    xlabel(ax, xlab, 'FontSize', 10);
    ylabel(ax, ylab, 'FontSize', 10);
    title(ax,  ttl,  'FontSize', title_fontsize, 'FontWeight', 'bold');
    set(ax, 'FontSize', 9);
end

% ─────────────────────────────────────────────────────────────

function plotZoomPanel(ax, t, speed, setpoint, ...
                       t_evt_start, t_evt_end, margin, ...
                       shade_clr, CLR, markEvent, ...
                       event_labels, event_clrs, panel_title)
%PLOTZOOMPANEL  Render a single zoomed disturbance subplot.
%
%   Shows the speed response over [t_evt_start-margin, t_evt_end+margin],
%   with a shaded background during the disturbance and event markers.

    y_top = max(setpoint) * 1.2;
    shadePeriod(ax, t_evt_start, t_evt_end, y_top, shade_clr);

    mask = (t >= t_evt_start - margin) & (t <= t_evt_end + margin);
    plot(ax, t(mask), setpoint(mask), '--', 'Color', CLR.ref,   'LineWidth', 2.0);
    hold(ax, 'on');
    plot(ax, t(mask), speed(mask),    '-',  'Color', CLR.speed,  'LineWidth', 2.5);

    markEvent(ax, t_evt_start, event_labels{1}, event_clrs(1,:));
    markEvent(ax, t_evt_end,   event_labels{2}, event_clrs(2,:));

    formatAxes(ax, 'Time (s)', 'Speed (km/h)', panel_title, 12);
    legend(ax, 'Setpoint', 'Actual speed', 'Location', 'best', 'FontSize', 9);
    xlim(ax, [t_evt_start - margin, t_evt_end + margin]);
end

% ─────────────────────────────────────────────────────────────

function printMetricLine(label, primary_val, primary_unit, ...
                          pct_val, pct_unit, limit_val, pass_label)
%PRINTMETRICLINE  Print a report row with optional PASS/FAIL indicator.
%
%   If primary_val is NaN, prints "N/A".
%   If pct_val is non-empty, shows both values; otherwise shows primary only.

    if isnan(primary_val)
        fprintf('║  %-24s : N/A                         ║\n', label);
        return;
    end

    if ~isempty(pct_val) && ~isnan(pct_val)
        fprintf('║  %-24s : %6.3f %-4s (%5.2f %s)   ║\n', ...
                label, primary_val, primary_unit, pct_val, pct_unit);
        ref = pct_val;
    else
        fprintf('║  %-24s : %6.2f %-4s                ║\n', ...
                label, primary_val, primary_unit);
        ref = primary_val;
    end

    if ref <= limit_val
        fprintf('║    -> PASS  (%s)                       ║\n', pass_label);
    else
        fprintf('║    -> FAIL  (%s)                       ║\n', pass_label);
    end
end

% ─────────────────────────────────────────────────────────────

function printValueLine(label, val, unit)
%PRINTVALUELINE  Print a report row with no PASS/FAIL requirement.
    if isnan(val)
        fprintf('║  %-30s : N/A          ║\n', label);
    else
        fprintf('║  %-30s : %6.2f %-5s ║\n', label, val, unit);
    end
end
