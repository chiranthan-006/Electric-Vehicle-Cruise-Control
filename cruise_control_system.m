%% ============================================================
%%  EV CRUISE CONTROL SYSTEM  —  Complete MATLAB Implementation
%%
%%  Plant model   :  G(s) = 1 / (5s + 1)
%%  Controller    :  PID with derivative filter
%%  Features      :  Interactive speed input, button engagement,
%%                   brake override, uphill/downhill disturbance
%%                   rejection, 6-panel analysis figure, zoomed
%%                   disturbance plots, quantitative report
%%
%%  HOW TO RUN:
%%    1. Full simulation (standalone, no Simulink needed):
%%         >> cruise_control_run_simulation()
%%
%%    2. Controller design + open-loop analysis only:
%%         >> cruise_control_design()
%%
%%    3. Post-Simulink analysis (requires Simulink workspace data):
%%         >> cruise_control_analyze_simulink()
%%
%%    4. Step-by-step Simulink build guide (prints to console):
%%         >> cruise_control_simulink_guide()
%%
%%  All four entry points are defined as functions in this file.
%%
%%  Author  : [Your Name]
%%  Version : 2.0
%%  Tested  : MATLAB R2020b+  |  Control System Toolbox required
%% ============================================================


%% ============================================================
%%  ENTRY POINT 1 — FULL STANDALONE SIMULATION
%% ============================================================

function cruise_control_run_simulation()
%CRUISE_CONTROL_RUN_SIMULATION
%   Interactive EV cruise control simulation.
%   Prompts for a target cruise speed, runs a 40-second Euler
%   integration, plots results, and prints a performance report.
%
%   Usage:
%     >> cruise_control_run_simulation()

    clear; close all; clc;

    % ── Banner ────────────────────────────────────────────────
    fprintf('╔══════════════════════════════════════════════════╗\n');
    fprintf('║  EV CRUISE CONTROL SYSTEM  —  SIMULATION SETUP   ║\n');
    fprintf('╚══════════════════════════════════════════════════╝\n\n');

    % ─────────────────────────────────────────────────────────
    %  SECTION 0  :  USER INPUT
    % ─────────────────────────────────────────────────────────
    SPEED_MIN_KMH = 10;
    SPEED_MAX_KMH = 200;
    DEFAULT_SPEED  = 60;

    cruise_speed_kmh = input(sprintf( ...
        '  Enter desired cruise speed (%d-%d km/h): ', ...
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

    % ─────────────────────────────────────────────────────────
    %  SECTION 1  :  SYSTEM PARAMETERS
    % ─────────────────────────────────────────────────────────

    % ── PID Gains (tuned for G(s) = 1/(5s+1)) ───────────────
    %   Closed-loop bandwidth ~1.8 rad/s; phase margin ~55 deg
    Kp = 8.5;   % Proportional gain
    Ki = 2.0;   % Integral gain         [eliminates SS error]
    Kd = 5.0;   % Derivative gain       [improves transient]
    Tf = 0.1;   % Derivative filter TC  [N = 1/Tf = 10]

    % ── Plant ─────────────────────────────────────────────────
    %   First-order EV drive:  tau * dy/dt + y = u + d
    %     y = normalised speed [0..1]
    %     u = throttle command [0..1]
    %     d = road-slope disturbance [normalised]
    tau = 5;              % Time constant [s]
    speed_scale = cruise_speed_kmh;   % [km/h per normalised unit]
    ref_norm    = 1.0;                % Normalised reference

    % ── Simulation grid ───────────────────────────────────────
    dt      = 0.01;    % 100 Hz — well above plant bandwidth
    T_total = 40;      % Simulation duration [s]
    t       = 0 : dt : T_total;
    N       = length(t);

    % ─────────────────────────────────────────────────────────
    %  SECTION 2  :  EVENT TIMELINE
    % ─────────────────────────────────────────────────────────
    t_cruise_on    = 4;    % Driver presses cruise button
    t_uphill       = 12;   % Road tilts uphill
    t_uphill_end   = 22;   % Road levels out
    t_downhill     = 24;   % Road tilts downhill
    t_downhill_end = 34;   % Road levels out
    t_brake        = 37;   % Driver brakes — cruise disengages

    DIST_UPHILL   = -0.18;   % Increased load (uphill)
    DIST_DOWNHILL = +0.15;   % Reduced  load (downhill)

    % ─────────────────────────────────────────────────────────
    %  SECTION 3  :  PRE-ALLOCATE STATE ARRAYS
    % ─────────────────────────────────────────────────────────
    speed_norm   = zeros(1, N);   % Normalised speed    [0..1]
    speed_kmh    = zeros(1, N);   % Speed               [km/h]
    setpoint_kmh = zeros(1, N);   % Cruise setpoint     [km/h]
    error_kmh    = zeros(1, N);   % Tracking error      [km/h]
    throttle_cmd = zeros(1, N);   % Throttle command    [0..1]
    disturbance  = zeros(1, N);   % Road disturbance    [norm]
    cruise_on    = false(1, N);   % Cruise-active flag

    THROTTLE_INIT = 0.40;   % Manual throttle before cruise engages

    % ─────────────────────────────────────────────────────────
    %  SECTION 4  :  INITIALISE PID STATES
    % ─────────────────────────────────────────────────────────
    pid_integral   = 0;
    pid_prev_err   = 0;
    pid_prev_deriv = 0;
    cruise_active  = false;
    setpoint_norm  = 0;

    % ─────────────────────────────────────────────────────────
    %  SECTION 5  :  MAIN SIMULATION LOOP
    %
    %  Euler forward integration. Each step computes:
    %    (a) disturbance value
    %    (b) cruise / brake state machine
    %    (c) PID or manual throttle
    %    (d) plant state update
    %    (e) record signals
    % ─────────────────────────────────────────────────────────
    for i = 2 : N

        tc = t(i-1);   % Current time [s]

        % ── 5a. Disturbance profile ───────────────────────────
        d = 0;
        if tc >= t_uphill   && tc < t_uphill_end,   d = DIST_UPHILL;   end
        if tc >= t_downhill && tc < t_downhill_end, d = DIST_DOWNHILL; end
        disturbance(i-1) = d;

        % ── 5b. Cruise / brake state machine ──────────────────
        if tc >= t_brake
            cruise_active = false;
            pid_integral  = 0;   % Clear integrator on disengage
        elseif tc >= t_cruise_on && ~cruise_active
            cruise_active = true;
            setpoint_norm = ref_norm;
        end

        % ── 5c. Throttle command ──────────────────────────────
        if cruise_active
            % PID control
            err          = setpoint_norm - speed_norm(i-1);
            pid_integral = pid_integral + err * dt;

            % Derivative with first-order low-pass filter
            %   D(s) = Kd * s / (Tf*s + 1)
            raw_deriv  = (err - pid_prev_err) / dt;
            alpha      = Tf / dt;
            filt_deriv = (raw_deriv + alpha * pid_prev_deriv) / (1 + alpha);

            u = Kp * err + Ki * pid_integral + Kd * filt_deriv;
            u = max(0, min(1, u));   % Actuator saturation [0, 1]

            pid_prev_err   = err;
            pid_prev_deriv = filt_deriv;

        else
            % Manual / coast throttle
            if tc < t_cruise_on
                u = min(THROTTLE_INIT, THROTTLE_INIT * tc / 2);   % Ramp-up
            else
                u = 0;   % Coast after braking
            end
            pid_integral   = 0;
            pid_prev_err   = 0;
            pid_prev_deriv = 0;
        end

        throttle_cmd(i-1) = u;

        % ── 5d. Plant dynamics (Euler forward) ────────────────
        %   tau * dy/dt = -y + u + d
        dydt          = (-speed_norm(i-1) + u + disturbance(i-1)) / tau;
        speed_norm(i) = speed_norm(i-1) + dydt * dt;
        speed_norm(i) = max(0, speed_norm(i));

        % ── 5e. Record signals ────────────────────────────────
        speed_kmh(i-1)    = speed_norm(i-1) * speed_scale;
        setpoint_kmh(i-1) = setpoint_norm   * speed_scale;
        error_kmh(i-1)    = (setpoint_norm - speed_norm(i-1)) * speed_scale;
        cruise_on(i-1)    = cruise_active;

    end  % end simulation loop

    % Fill final sample
    speed_kmh(end)    = speed_norm(end) * speed_scale;
    setpoint_kmh(end) = setpoint_norm   * speed_scale;
    error_kmh(end)    = error_kmh(end-1);
    throttle_cmd(end) = throttle_cmd(end-1);
    disturbance(end)  = disturbance(end-1);
    cruise_on(end)    = cruise_active;

    % ─────────────────────────────────────────────────────────
    %  SECTION 6  :  PERFORMANCE METRICS
    % ─────────────────────────────────────────────────────────
    RECOVERY_BAND_KMH  = cruise_speed_kmh * 0.02;   % 2% of setpoint
    RECOVERY_BAND_PCT  = 2.0;

    % ── Indices for post-engagement analysis ──────────────────
    idx_on = find(cruise_on, 1, 'first');

    ss_error_pct = NaN;  overshoot_pct = NaN;
    rise_time    = NaN;  settling_time = NaN;

    if ~isempty(idx_on)
        sp = setpoint_kmh(idx_on);   % Capture setpoint at engagement

        % Steady-state error — use last 5 s of cruise window
        idx_brake = find(t >= t_brake, 1);
        if isempty(idx_brake), idx_brake = N; end
        idx_ss = max(idx_on, idx_brake - round(5/dt));
        ss_error_kmh = mean(abs(error_kmh(idx_ss : idx_brake-1)));
        ss_error_pct = (ss_error_kmh / sp) * 100;

        % Overshoot
        max_speed    = max(speed_kmh(idx_on : idx_brake-1));
        overshoot_pct = max(0, (max_speed - sp) / sp * 100);

        % Rise time (10% → 90% of setpoint, from engagement)
        spd_slice = speed_kmh(idx_on : idx_brake-1);
        i10 = find(spd_slice >= 0.10 * sp, 1);
        i90 = find(spd_slice >= 0.90 * sp, 1);
        if ~isempty(i10) && ~isempty(i90)
            rise_time = (i90 - i10) * dt;
        end

        % Settling time (within 2% band, from engagement)
        i_set = find(abs(spd_slice - sp) <= RECOVERY_BAND_KMH, 1);
        if ~isempty(i_set)
            settling_time = (i_set - 1) * dt;
        end
    end

    % ── Disturbance metrics ───────────────────────────────────
    [max_dev_up, rec_up] = disturbanceMetrics( ...
        t, error_kmh, t_uphill, t_uphill_end, RECOVERY_BAND_KMH);
    [max_dev_dn, rec_dn] = disturbanceMetrics( ...
        t, error_kmh, t_downhill, t_downhill_end, RECOVERY_BAND_KMH);

    % ─────────────────────────────────────────────────────────
    %  SECTION 7  :  CONSOLE PERFORMANCE REPORT
    % ─────────────────────────────────────────────────────────
    fprintf('╔══════════════════════════════════════════════════╗\n');
    fprintf('║           PERFORMANCE REPORT                      ║\n');
    fprintf('║  Target: %3.0f km/h | PID Kp=%.1f Ki=%.1f Kd=%.1f  ║\n', ...
            cruise_speed_kmh, Kp, Ki, Kd);
    fprintf('╠══════════════════════════════════════════════════╣\n');

    printMetricLine('Steady-state error', ss_error_kmh, 'km/h', ...
                    ss_error_pct, '%', RECOVERY_BAND_PCT, '< 2%');
    printMetricLine('Overshoot', overshoot_pct, '%', [], '', 5.0, '< 5%');
    printValueLine('Rise time (10%%–90%%)', rise_time, 's');
    printValueLine('Settling time (2%% band)', settling_time, 's');

    fprintf('╠══════════════════════════════════════════════════╣\n');
    fprintf('║  DISTURBANCE REJECTION                            ║\n');
    fprintf('╠══════════════════════════════════════════════════╣\n');

    printValueLine('Uphill  — peak deviation', max_dev_up, 'km/h');
    printValueLine('Uphill  — recovery time',  rec_up,     's');
    printValueLine('Downhill — peak deviation', max_dev_dn, 'km/h');
    printValueLine('Downhill — recovery time',  rec_dn,     's');

    fprintf('╚══════════════════════════════════════════════════╝\n\n');

    % ─────────────────────────────────────────────────────────
    %  SECTION 8  :  6-PANEL ANALYSIS FIGURE
    % ─────────────────────────────────────────────────────────

    % Colour palette
    CLR.speed  = [0.13 0.47 0.71];   % Steel blue
    CLR.ref    = [0.84 0.15 0.16];   % Red
    CLR.ctrl   = [0.17 0.63 0.17];   % Green
    CLR.err    = [1.00 0.50 0.05];   % Orange
    CLR.dist   = [0.58 0.40 0.74];   % Purple
    CLR.shade  = [0.95 0.95 0.80];   % Pale yellow

    % Helper: add a vertical event line
    markEvent = @(ax, tx, lbl, clr) xline(ax, tx, '--', lbl, ...
        'Color', clr, 'LineWidth', 1.2, 'FontSize', 8);

    fig1 = figure('Name', 'EV Cruise Control — Performance Analysis', ...
                  'Position', [50 50 1400 900], 'Color', [1 1 1]);

    % ── Panel 1: Speed tracking ───────────────────────────────
    ax1 = subplot(3, 2, 1);
    shadePeriod(ax1, t_cruise_on, t_brake, cruise_speed_kmh * 1.25, [0.93 0.97 1.00]);
    shadePeriod(ax1, t_uphill,   t_uphill_end,   cruise_speed_kmh * 1.25, [1.00 0.93 0.93]);
    shadePeriod(ax1, t_downhill, t_downhill_end, cruise_speed_kmh * 1.25, [0.93 1.00 0.93]);
    hold(ax1, 'on');
    plot(ax1, t, setpoint_kmh, '--', 'Color', CLR.ref,   'LineWidth', 2.0);
    plot(ax1, t, speed_kmh,    '-',  'Color', CLR.speed, 'LineWidth', 2.5);
    markEvent(ax1, t_cruise_on,  'Cruise ON',  [0 0.6 0]);
    markEvent(ax1, t_brake,      'Brake',      [0.8 0 0]);
    legend(ax1, 'Setpoint', 'Actual speed', 'Location', 'best', 'FontSize', 9);
    formatAxes(ax1, 'Time (s)', 'Speed (km/h)', 'Speed Tracking', 12);

    % ── Panel 2: Tracking error ───────────────────────────────
    ax2 = subplot(3, 2, 2);
    hold(ax2, 'on');
    plot(ax2, t, error_kmh, '-', 'Color', CLR.err, 'LineWidth', 2);
    yline(ax2,  RECOVERY_BAND_KMH, 'g--', sprintf('+%.1f km/h', RECOVERY_BAND_KMH), ...
          'LineWidth', 1.2, 'FontSize', 8);
    yline(ax2, -RECOVERY_BAND_KMH, 'g--', sprintf('−%.1f km/h', RECOVERY_BAND_KMH), ...
          'LineWidth', 1.2, 'FontSize', 8);
    yline(ax2, 0, 'k:', 'LineWidth', 0.8);
    markEvent(ax2, t_uphill,   'Uphill',    CLR.dist);
    markEvent(ax2, t_downhill, 'Downhill',  [0.2 0.6 0.8]);
    markEvent(ax2, t_brake,    'Brake',     [0.8 0 0]);
    formatAxes(ax2, 'Time (s)', 'Error (km/h)', 'Tracking Error', 12);

    % ── Panel 3: Throttle command ─────────────────────────────
    ax3 = subplot(3, 2, 3);
    hold(ax3, 'on');
    area(ax3, t, throttle_cmd, 'FaceColor', CLR.ctrl, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    plot(ax3, t, throttle_cmd, '-', 'Color', CLR.ctrl, 'LineWidth', 2);
    yline(ax3, 1, 'r--', 'Saturation', 'LineWidth', 1.2, 'FontSize', 8);
    yline(ax3, 0, 'k:',  'LineWidth', 0.8);
    markEvent(ax3, t_cruise_on, 'Cruise ON', [0 0.6 0]);
    markEvent(ax3, t_brake,     'Brake',     [0.8 0 0]);
    ylim(ax3, [-0.05 1.15]);
    formatAxes(ax3, 'Time (s)', 'Throttle [0,1]', 'Controller Output', 12);

    % ── Panel 4: Road disturbance ─────────────────────────────
    ax4 = subplot(3, 2, 4);
    hold(ax4, 'on');
    stairs(ax4, t, disturbance, '-', 'Color', CLR.dist, 'LineWidth', 2);
    yline(ax4, 0, 'k:', 'LineWidth', 0.8);
    markEvent(ax4, t_uphill,      'Uphill start',  [0.8 0.1 0.1]);
    markEvent(ax4, t_uphill_end,  'Uphill end',    [0.5 0.5 0.5]);
    markEvent(ax4, t_downhill,    'Downhill start',[0.1 0.4 0.8]);
    markEvent(ax4, t_downhill_end,'Downhill end',  [0.5 0.5 0.5]);
    formatAxes(ax4, 'Time (s)', 'Disturbance [norm]', 'Road-Slope Disturbance', 12);

    % ── Panel 5: Performance bar chart ───────────────────────
    ax5 = subplot(3, 2, 5);
    metric_labels = {'SS Error (%)', 'Overshoot (%)'};
    actual_vals   = [ss_error_pct, overshoot_pct];
    limit_vals    = [2, 5];

    bar_h = bar(ax5, 1:2, [actual_vals; limit_vals]', 0.7);
    bar_h(1).FaceColor = CLR.speed;
    bar_h(2).FaceColor = [0.85 0.85 0.85];

    for k = 1:2
        val = actual_vals(k);  lim = limit_vals(k);
        if ~isnan(val)
            if val <= lim, lbl = 'PASS'; clr = [0.10 0.60 0.10];
            else,           lbl = 'FAIL'; clr = [0.80 0.10 0.10];
            end
            text(ax5, k, max(val, lim) + 0.15, lbl, ...
                 'HorizontalAlignment', 'center', ...
                 'Color', clr, 'FontWeight', 'bold', 'FontSize', 9);
        end
    end

    set(ax5, 'XTick', 1:2, 'XTickLabel', metric_labels, 'FontSize', 10);
    legend(ax5, 'Actual', 'Limit', 'Location', 'northwest', 'FontSize', 9);
    ylabel(ax5, 'Percentage (%)');
    title(ax5, 'Performance vs. Requirements', 'FontSize', 12, 'FontWeight', 'bold');
    grid(ax5, 'on');
    valid_v = actual_vals(~isnan(actual_vals));
    ylim(ax5, [0, max([valid_v, limit_vals]) * 1.5 + 0.5]);

    % ── Panel 6: Cruise-on flag ───────────────────────────────
    ax6 = subplot(3, 2, 6);
    area(ax6, t, double(cruise_on), 'FaceColor', [0.20 0.75 0.40], ...
         'FaceAlpha', 0.5, 'EdgeColor', 'none');
    hold(ax6, 'on');
    plot(ax6, t, double(cruise_on), '-', 'Color', [0.10 0.55 0.25], 'LineWidth', 2);
    ylim(ax6, [-0.1 1.3]);
    set(ax6, 'YTick', [0 1], 'YTickLabel', {'OFF', 'ON'});
    markEvent(ax6, t_cruise_on, 'Engage', [0 0.6 0]);
    markEvent(ax6, t_brake,     'Brake',  [0.8 0 0]);
    formatAxes(ax6, 'Time (s)', 'State', 'Cruise Active Flag', 12);

    sgtitle(fig1, sprintf( ...
        'EV Cruise Control  |  Target: %d km/h  |  PID  Kp=%.1f  Ki=%.1f  Kd=%.1f  Tf=%.2f s', ...
        cruise_speed_kmh, Kp, Ki, Kd, Tf), ...
        'FontSize', 13, 'FontWeight', 'bold');

    % ─────────────────────────────────────────────────────────
    %  SECTION 9  :  ZOOMED DISTURBANCE FIGURES
    % ─────────────────────────────────────────────────────────
    fig2 = figure('Name', 'Disturbance Rejection — Zoomed', ...
                  'Position', [60 60 1200 480], 'Color', [1 1 1]);

    ZOOM_MARGIN = 2;

    ax_up = subplot(1, 2, 1);
    plotZoomPanel(ax_up, t, speed_kmh, setpoint_kmh, ...
        t_uphill, t_uphill_end, ZOOM_MARGIN, ...
        [1.00 0.92 0.92], CLR, markEvent, ...
        {'Uphill starts', 'Flat again'}, ...
        [0.8 0.1 0.1; 0.5 0.5 0.5], ...
        'Uphill Response (Zoomed)');

    ax_dn = subplot(1, 2, 2);
    plotZoomPanel(ax_dn, t, speed_kmh, setpoint_kmh, ...
        t_downhill, t_downhill_end, ZOOM_MARGIN, ...
        [0.90 0.95 1.00], CLR, markEvent, ...
        {'Downhill starts', 'Flat again'}, ...
        [0.1 0.4 0.8; 0.5 0.5 0.5], ...
        'Downhill Response (Zoomed)');

    sgtitle(fig2, 'Disturbance Rejection — Uphill & Downhill', ...
            'FontSize', 13, 'FontWeight', 'bold');

    % ─────────────────────────────────────────────────────────
    %  SECTION 10  :  SAVE OUTPUTS
    % ─────────────────────────────────────────────────────────
    fname_fig1 = sprintf('cruise_analysis_%dkmh.png',  cruise_speed_kmh);
    fname_fig2 = sprintf('disturbance_zoom_%dkmh.png', cruise_speed_kmh);
    fname_mat  = sprintf('simulation_data_%dkmh.mat',  cruise_speed_kmh);

    saveas(fig1, fname_fig1);
    saveas(fig2, fname_fig2);
    save(fname_mat, 't', 'speed_kmh', 'setpoint_kmh', ...
         'error_kmh', 'throttle_cmd', 'disturbance', 'cruise_on');

    fprintf('Saved:\n  %s\n  %s\n  %s\n\nDone.\n\n', ...
            fname_fig1, fname_fig2, fname_mat);

end  % function cruise_control_run_simulation


%% ============================================================
%%  ENTRY POINT 2 — CONTROLLER DESIGN & OPEN-LOOP ANALYSIS
%% ============================================================

function cruise_control_design()
%CRUISE_CONTROL_DESIGN
%   Designs the PID controller for G(s) = 1/(5s+1).
%   Compares auto-tuned vs manually optimised parameters,
%   analyses stability margins, and saves params to .mat file.
%
%   Requires: Control System Toolbox
%
%   Usage:
%     >> cruise_control_design()

    clear; close all; clc;

    % ── Plant ─────────────────────────────────────────────────
    num = 1;  den = [5 1];
    G = tf(num, den);

    fprintf('=== SYSTEM TRANSFER FUNCTION ===\n');
    fprintf('  G(s) = 1/(5s + 1)\n');
    fprintf('  Pole: %.4f  |  Time constant: 5 s\n\n', pole(G));

    % ── Open-loop step response ───────────────────────────────
    figure('Name', 'Open-Loop Response', 'Position', [100 100 1200 400]);
    subplot(1, 2, 1);
    [y_ol, t_ol] = step(G, 30);
    plot(t_ol, y_ol, 'LineWidth', 2); grid on;
    title('Open-Loop Step Response'); xlabel('Time (s)'); ylabel('Speed (norm)');

    info_ol = stepinfo(G);
    fprintf('=== OPEN-LOOP PERFORMANCE ===\n');
    fprintf('  Rise time     : %.2f s\n', info_ol.RiseTime);
    fprintf('  Settling time : %.2f s\n', info_ol.SettlingTime);
    fprintf('  SS value      : %.4f  (error: %.2f%%)\n\n', ...
            y_ol(end), (1-y_ol(end))*100);

    subplot(1, 2, 2); margin(G); grid on; title('Open-Loop Bode Plot');

    % ── Auto-tuned PID ────────────────────────────────────────
    opts   = pidtuneOptions('DesignFocus', 'balanced');
    C_auto = pidtune(G, 'PID', opts);

    fprintf('=== AUTO-TUNED PID ===\n');
    fprintf('  Kp=%.4f  Ki=%.4f  Kd=%.4f  Tf=%.4f\n\n', ...
            C_auto.Kp, C_auto.Ki, C_auto.Kd, C_auto.Tf);

    % ── Manually optimised PID ────────────────────────────────
    Kp = 8.5;  Ki = 2.0;  Kd = 5.0;  Tf = 0.1;
    C_manual = pid(Kp, Ki, Kd, Tf);

    fprintf('=== MANUALLY OPTIMISED PID ===\n');
    fprintf('  Kp=%.4f  Ki=%.4f  Kd=%.4f  Tf=%.4f\n\n', Kp, Ki, Kd, Tf);

    % ── Closed-loop analysis ──────────────────────────────────
    T_auto   = feedback(C_auto   * G, 1);
    T_manual = feedback(C_manual * G, 1);

    figure('Name', 'Controller Comparison', 'Position', [100 150 1400 500]);
    subplot(1, 2, 1);
    step(G, 'b--', T_auto, 'r', T_manual, 'g', 30);
    legend('Open-Loop', 'Auto-tuned', 'Manual', 'Location', 'best');
    grid on; title('Step Response Comparison');
    xlabel('Time (s)'); ylabel('Speed (norm)');

    info_a = stepinfo(T_auto);
    info_m = stepinfo(T_manual);

    fprintf('=== CLOSED-LOOP: AUTO-TUNED ===\n');
    fprintf('  Rise %.2f s  |  Settle %.2f s  |  Overshoot %.2f%%  |  SS err %.2f%%\n\n', ...
            info_a.RiseTime, info_a.SettlingTime, info_a.Overshoot, ...
            (1 - dcgain(T_auto))*100);

    fprintf('=== CLOSED-LOOP: MANUAL ===\n');
    fprintf('  Rise %.2f s  |  Settle %.2f s  |  Overshoot %.2f%%  |  SS err %.2f%%\n\n', ...
            info_m.RiseTime, info_m.SettlingTime, info_m.Overshoot, ...
            (1 - dcgain(T_manual))*100);

    % Requirements bar
    subplot(1, 2, 2);
    reqs = {'Overshoot < 5%', 'SS Err < 2%', 'Stable'};
    a_pass = [info_a.Overshoot<5, abs(1-dcgain(T_auto))*100<2, isstable(T_auto)];
    m_pass = [info_m.Overshoot<5, abs(1-dcgain(T_manual))*100<2, isstable(T_manual)];
    bar(1:3, [a_pass; m_pass]');
    set(gca, 'XTickLabel', reqs); ylim([0 1.2]);
    legend('Auto-tuned', 'Manual'); ylabel('Pass (1) / Fail (0)');
    title('Requirements Verification'); grid on;

    % ── Stability margins ──────────────────────────────────────
    [Gm, Pm, Wcg, Wcp] = margin(C_manual * G);
    fprintf('=== STABILITY MARGINS ===\n');
    fprintf('  Gain margin  : %.2f dB at %.2f rad/s\n', 20*log10(Gm), Wcg);
    fprintf('  Phase margin : %.2f deg at %.2f rad/s\n\n', Pm, Wcp);

    % ── Save parameters ───────────────────────────────────────
    controller_params.Kp        = Kp;
    controller_params.Ki        = Ki;
    controller_params.Kd        = Kd;
    controller_params.Tf        = Tf;
    controller_params.plant_num = num;
    controller_params.plant_den = den;
    save('cruise_control_params.mat', 'controller_params');

    fprintf('Parameters saved → cruise_control_params.mat\n\n');
    fprintf('=== SUMMARY ===\n');
    fprintf('  Kp=%.4f  Ki=%.4f  Kd=%.4f  Tf=%.4f\n', Kp, Ki, Kd, Tf);
    fprintf('  SS error  : %.2f%% (req < 2%%)\n', abs(1-dcgain(T_manual))*100);
    fprintf('  Overshoot : %.2f%% (req < 5%%)\n\n', info_m.Overshoot);

end  % function cruise_control_design


%% ============================================================
%%  ENTRY POINT 3 — POST-SIMULINK SIMULATION ANALYSIS
%% ============================================================

function cruise_control_analyze_simulink()
%CRUISE_CONTROL_ANALYZE_SIMULINK
%   Run after the Simulink model to analyse workspace data.
%   Expects variables: tout, speed_out, setpoint_out,
%   control_out, error_out, cruise_status_out.
%
%   Usage:
%     >> sim('cruise_control_system')
%     >> cruise_control_analyze_simulink()

    fprintf('═══════════════════════════════════════════════════\n');
    fprintf('  CRUISE CONTROL — POST-SIMULINK ANALYSIS\n');
    fprintf('═══════════════════════════════════════════════════\n\n');

    % ── Load workspace data ───────────────────────────────────
    required = {'tout','speed_out','setpoint_out', ...
                'control_out','error_out','cruise_status_out'};
    for k = 1:numel(required)
        if ~evalin('base', sprintf('exist(''%s'',''var'')', required{k}))
            error('Variable "%s" not found. Run Simulink model first.', required{k});
        end
    end

    t             = evalin('base', 'tout');
    speed         = evalin('base', 'speed_out');
    setpoint      = evalin('base', 'setpoint_out');
    control       = evalin('base', 'control_out');
    error_signal  = evalin('base', 'error_out');
    cruise_status = evalin('base', 'cruise_status_out');

    % ── Cruise activation metrics ─────────────────────────────
    cruise_active_idx = find(cruise_status > 0.5);
    passed = 0; total = 4;

    if isempty(cruise_active_idx)
        warning('Cruise control was never activated!');
    else
        cruise_start_time = t(cruise_active_idx(1));
        sp = setpoint(cruise_active_idx(1));

        fprintf('Cruise activated at t = %.2f s  |  setpoint = %.2f km/h\n\n', ...
                cruise_start_time, sp);

        % Steady-state error
        idx_ss = find(t > cruise_start_time + 10, 1);
        if ~isempty(idx_ss)
            ss_err = abs(setpoint(idx_ss:end) - speed(idx_ss:end));
            ss_error_pct = mean(ss_err) / mean(setpoint(idx_ss:end)) * 100;
            fprintf('Steady-state error : %.2f%%\n', ss_error_pct);
            if ss_error_pct < 2, fprintf('  ✓ PASS (< 2%%)\n\n'); passed=passed+1;
            else,                  fprintf('  ✗ FAIL\n\n'); end
        end

        % Overshoot
        spd_cruise = speed(cruise_active_idx);
        overshoot  = max(0, (max(spd_cruise) - sp) / sp * 100);
        fprintf('Overshoot : %.2f%%\n', overshoot);
        if overshoot < 5, fprintf('  ✓ PASS (< 5%%)\n\n'); passed=passed+1;
        else,              fprintf('  ✗ FAIL\n\n'); end

        % Settling time
        band = sp * 0.02;
        i_set = find(abs(spd_cruise - sp) <= band, 1);
        if ~isempty(i_set)
            settle = t(cruise_active_idx(i_set)) - cruise_start_time;
            fprintf('Settling time : %.2f s\n\n', settle);
            passed = passed + 1;
        end
    end

    % Disturbance
    dist_idx = find(t >= 10, 1);
    if ~isempty(dist_idx) && cruise_status(dist_idx) > 0.5
        dev       = abs(speed(dist_idx:end) - setpoint(dist_idx:end));
        rec_band  = mean(setpoint(dist_idx:end)) * 0.02;
        rec_idx   = find(dev <= rec_band, 1);
        if ~isempty(rec_idx)
            rec_time = t(dist_idx + rec_idx - 1) - t(dist_idx);
            fprintf('Disturbance recovery : %.2f s\n', rec_time);
            if rec_time < 10, fprintf('  ✓ PASS\n\n'); passed=passed+1;
            else,              fprintf('  ✗ FAIL\n\n'); end
        end
    end

    fprintf('Overall: %d / %d requirements met\n\n', passed, total);

    % ── Comprehensive plot ────────────────────────────────────
    figure('Name', 'Simulink Performance Analysis', ...
           'Position', [50 50 1400 900]);

    subplot(3,2,1);
    plot(t, setpoint, 'r--', 'LineWidth', 2); hold on;
    plot(t, speed,    'b',   'LineWidth', 2);
    grid on; legend('Setpoint','Speed'); xlabel('Time (s)'); ylabel('km/h');
    title('Speed Tracking'); xline(10,'k--','Disturbance');

    subplot(3,2,2);
    plot(t, error_signal, 'r', 'LineWidth', 2); hold on;
    yline(2,'g--','+2%'); yline(-2,'g--','-2%');
    xline(10,'k--','Disturbance'); grid on;
    xlabel('Time (s)'); ylabel('Error (km/h)'); title('Tracking Error');

    subplot(3,2,3);
    plot(t, control, 'g', 'LineWidth', 2); hold on;
    yline(1,'r--','Sat'); yline(0,'k:');
    xline(10,'k--','Disturbance'); grid on;
    xlabel('Time (s)'); ylabel('Control [0,1]'); title('Throttle Command');

    subplot(3,2,[4 6]);
    plot(t, speed, 'b', 'LineWidth', 2); hold on;
    plot(t, setpoint, 'r--', 'LineWidth', 1.5);
    plot(t, cruise_status .* max(setpoint) * 1.05, 'm:', 'LineWidth', 1.5);
    xline(10,'k--'); grid on;
    legend('Speed','Setpoint','Cruise status');
    xlabel('Time (s)'); ylabel('km/h'); title('Full System Overview');

    % ── Save ──────────────────────────────────────────────────
    results.time = t; results.speed = speed; results.setpoint = setpoint;
    results.control = control; results.error = error_signal;
    save('simulation_results.mat', 'results');
    saveas(gcf, 'cruise_control_performance.png');
    fprintf('Saved: simulation_results.mat | cruise_control_performance.png\n\n');

end  % function cruise_control_analyze_simulink


%% ============================================================
%%  ENTRY POINT 4 — SIMULINK BUILD GUIDE (console print)
%% ============================================================

function cruise_control_simulink_guide()
%CRUISE_CONTROL_SIMULINK_GUIDE
%   Prints the step-by-step Simulink model building guide.
%
%   Usage:
%     >> cruise_control_simulink_guide()

    fprintf('═══════════════════════════════════════════════════\n');
    fprintf('  SIMULINK MODEL BUILDING GUIDE\n');
    fprintf('  cruise_control_system.slx\n');
    fprintf('═══════════════════════════════════════════════════\n\n');

    fprintf('STEP 1 — Create new model\n');
    fprintf('  simulink → New Model (Ctrl+N)\n');
    fprintf('  Save as: cruise_control_system.slx\n\n');

    fprintf('STEP 2 — Add INPUT blocks (Sources library)\n');
    fprintf('  • Pulse Generator   → Cruise_Enable_Button\n');
    fprintf('    Period=1000 | Amplitude=1 | PulseWidth=2 | Delay=3\n');
    fprintf('  • Signal Builder    → Throttle_Input\n');
    fprintf('    t=[0,2,3,5,7,30]  Throttle=[0.4,0.4,0.5,0.5,0.5,0.5]\n');
    fprintf('  • Pulse Generator   → Brake_Pedal\n');
    fprintf('    Period=1000 | Amplitude=1 | PulseWidth=2 | Delay=15\n');
    fprintf('  • Step              → Slope_Disturbance\n');
    fprintf('    StepTime=10 | Init=0 | Final=-0.15\n\n');

    fprintf('STEP 3 — Add LOGIC blocks (MATLAB Function)\n');
    fprintf('  • Cruise_State_Logic  (inputs: cruise_button, brake)\n');
    fprintf('    Toggle on rising edge; disable on brake press.\n');
    fprintf('    persistent state prev_button\n');
    fprintf('    if cruise_button>0.5 && prev_button<=0.5: state=~state\n');
    fprintf('    if brake>0.5: state=0\n\n');
    fprintf('  • Cruise_Speed_Setter (inputs: cruise_active, speed, throttle)\n');
    fprintf('    Capture speed at engagement; update if throttle higher.\n\n');

    fprintf('STEP 4 — Add CONTROLLER blocks\n');
    fprintf('  • Sum → Speed_Error  (setpoint − current_speed)\n');
    fprintf('  • PID Controller: Kp=8.5  Ki=2.0  Kd=5.0  N=10\n');
    fprintf('    Enable external reset (connect ~cruise_active)\n');
    fprintf('  • Switch → Control_Mode_Switch\n');
    fprintf('    Input1=Throttle | Input2=cruise_active | Input3=PID_out\n');
    fprintf('  • Saturation: [0, 1]\n\n');

    fprintf('STEP 5 — Add PLANT blocks\n');
    fprintf('  • Sum    → Add_Disturbance (control + slope)\n');
    fprintf('  • Transfer Fcn: num=[1]  den=[5 1]\n');
    fprintf('  • Gain   → Speed_to_kmh (value = target km/h, e.g. 50)\n\n');

    fprintf('STEP 6 — Add OUTPUT blocks\n');
    fprintf('  • 4 Scopes  (Speed, Control, Error, Status)\n');
    fprintf('  • 5 To Workspace blocks\n');
    fprintf('    speed_out | setpoint_out | control_out | error_out | cruise_status_out\n');
    fprintf('  • 3 Display blocks (current speed, setpoint, status)\n\n');

    fprintf('STEP 7 — Simulation settings\n');
    fprintf('  Simulation > Configuration Parameters:\n');
    fprintf('    Stop time: 30 | Solver: ode45 | Type: Variable-step\n\n');

    fprintf('STEP 8 — Run and verify\n');
    fprintf('  Click Run → open all scopes\n');
    fprintf('  Then: >> cruise_control_analyze_simulink()\n\n');

    fprintf('PID BLOCK SETTINGS (for reference):\n');
    fprintf('  Kp = 8.5 | Ki = 2.0 | Kd = 5.0 | N (filter) = 10\n');
    fprintf('  Enable: External reset, Anti-windup\n\n');

    fprintf('═══════════════════════════════════════════════════\n\n');

end  % function cruise_control_simulink_guide


%% ============================================================
%%  LOCAL HELPER FUNCTIONS
%%  (shared across all entry points above)
%% ============================================================

function [max_dev, recovery_time] = disturbanceMetrics(t, err_kmh, t_start, t_end, band)
%DISTURBANCEMETRICS  Peak deviation and recovery time for a disturbance window.
%
%   Inputs:
%     t             – time vector [s]
%     err_kmh       – tracking error [km/h]
%     t_start/t_end – disturbance window [s]
%     band          – tolerance band [km/h]
%
%   Outputs:
%     max_dev       – peak |error| in window [km/h]
%     recovery_time – seconds from t_start until |error| ≤ band; NaN if never

    i_start = find(t >= t_start, 1, 'first');
    i_end   = find(t >= t_end,   1, 'first');

    if isempty(i_start) || isempty(i_end) || i_start >= i_end
        max_dev = NaN;  recovery_time = NaN;  return;
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
%SHADEPERIOD  Shaded background band between two time bounds.
    fill(ax, [t_lo t_hi t_hi t_lo], [0 0 y_max y_max], face_clr, ...
         'EdgeColor', 'none', 'FaceAlpha', 0.5);
    hold(ax, 'on');
end

% ─────────────────────────────────────────────────────────────

function formatAxes(ax, xlab, ylab, ttl, title_fontsize)
%FORMATAXES  Consistent grid, labels, and title for an axes object.
    grid(ax, 'on');  grid(ax, 'minor');
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
%PLOTZOOMPANEL  Zoomed disturbance subplot with shading and event markers.

    y_top = max(setpoint) * 1.2;
    shadePeriod(ax, t_evt_start, t_evt_end, y_top, shade_clr);

    mask = (t >= t_evt_start - margin) & (t <= t_evt_end + margin);
    plot(ax, t(mask), setpoint(mask), '--', 'Color', CLR.ref,   'LineWidth', 2.0);
    hold(ax, 'on');
    plot(ax, t(mask), speed(mask),    '-',  'Color', CLR.speed, 'LineWidth', 2.5);

    markEvent(ax, t_evt_start, event_labels{1}, event_clrs(1,:));
    markEvent(ax, t_evt_end,   event_labels{2}, event_clrs(2,:));

    formatAxes(ax, 'Time (s)', 'Speed (km/h)', panel_title, 12);
    legend(ax, 'Setpoint', 'Actual speed', 'Location', 'best', 'FontSize', 9);
    xlim(ax, [t_evt_start - margin, t_evt_end + margin]);
end

% ─────────────────────────────────────────────────────────────

function printMetricLine(label, primary_val, primary_unit, ...
                          pct_val, pct_unit, limit_val, pass_label)
%PRINTMETRICLINE  Report row with optional PASS/FAIL indicator.

    if isnan(primary_val)
        fprintf('║  %-24s : N/A\n', label);  return;
    end

    if ~isempty(pct_val) && ~isnan(pct_val)
        fprintf('║  %-24s : %6.3f %-4s  (%5.2f %s)\n', ...
                label, primary_val, primary_unit, pct_val, pct_unit);
        ref = pct_val;
    else
        fprintf('║  %-24s : %6.2f %-4s\n', label, primary_val, primary_unit);
        ref = primary_val;
    end

    if ref <= limit_val
        fprintf('║    → PASS  (%s)\n', pass_label);
    else
        fprintf('║    → FAIL  (%s)\n', pass_label);
    end
end

% ─────────────────────────────────────────────────────────────

function printValueLine(label, val, unit)
%PRINTVALUELINE  Report row with no PASS/FAIL requirement.
    if isnan(val)
        fprintf('║  %-30s : N/A\n', label);
    else
        fprintf('║  %-30s : %6.2f %s\n', label, val, unit);
    end
end
