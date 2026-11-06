%LOOP_DESIGN  Outer tracking-loop design on the identified servo model, and the effect of delay.
%
%   Needs servo_model_<axis>.mat from servo_sysid.m (or set USE_PLACEHOLDER = true to run
%   with a placeholder model before the servo has been measured).
%
%   Part A  v1 firmware structure (pixel-error feedback):
%           goal += KP*e + KD*de at 500 Hz  ==  PI controller  C(s) = KD' + KP'*fs/s
%           -> PI gains for 60 deg phase margin at each vision delay, and the firmware constants
%   Part B  V2 structure (absolute LOS + CV prediction -> goal = predicted target angle)
%           -> RMS tracking error vs total vision delay: feedback (C0) vs prediction (C2)
%           This is the simulation half of the latency paper (A안): predict, then measure.
%
%   Requires Control System Toolbox.

if ~exist('USE_PLACEHOLDER', 'var'), USE_PLACEHOLDER = false; end
if ~exist('AXIS', 'var'),            AXIS = 'pan';            end

%% parameters (edit from field runs / sysid)
fs        = 500;                 % STM32 loop [Hz]
Ts        = 1 / fs;
f_vision  = 120;                 % camera / vision update [Hz] (run 2 achieved 60)
Tv_meas   = 0.035;               % measured vision delay: exposure mid -> command at STM32 [s]
Tv_list   = (5:5:60) * 1e-3;     % sweep for the bandwidth / error curves [s]
sigma_los = 0.06;                % LOS measurement noise, 1 sigma [deg] (~1 mrad)
sigma_acc = 300;                 % CV filter process noise: target acceleration, 1 sigma [deg/s^2]
PX_PER_DEG_V1 = 1920 / 102;      % v1 virtual 1920 px over the 102 deg wide-camera FOV
C2D = 360 / 4096;                % servo counts -> deg

%% servo model
if USE_PLACEHOLDER
    % placeholder: 2nd order, 8 Hz, zeta 0.7, 6 ms dead time (replace by the identified model)
    wn = 2*pi*8; G = tf(wn^2, [1 2*0.7*wn wn^2], 'IODelay', 0.006);
    warning('loop_design:placeholder', 'using a PLACEHOLDER servo model');
else
    S = load(['servo_model_' AXIS '.mat']);
    G = S.G;
end
s = tf('s');

%% Part A: PI design vs vision delay (v1 structure)
% vision sample-and-hold adds ~half a frame period of delay on average
Tzoh = 0.5 / f_vision;
optsPI = pidtuneOptions('PhaseMargin', 60);
A = table('Size', [numel(Tv_list) 6], 'VariableTypes', repmat({'double'}, 1, 6), ...
          'VariableNames', {'Tv_ms', 'Kp', 'Ki', 'wc_Hz', 'PM_deg', 'GM_dB'});
for i = 1:numel(Tv_list)
    P = G * exp(-s * (Tv_list(i) + Tzoh));
    [C, info] = pidtune(P, 'PI', optsPI);
    [Gm, Pm] = margin(C * P);
    A(i, :) = {1e3 * Tv_list(i), C.Kp, C.Ki, info.CrossoverFrequency / (2*pi), Pm, 20*log10(Gm)};
end
disp('=== Part A: PI (60 deg PM) vs vision delay ==='); disp(A);

[Cm, infom] = pidtune(G * exp(-s * (Tv_meas + Tzoh)), 'PI', optsPI);
% map to v1 firmware constants:  goal_counts += KP*e_px every tick,  + KD*de_px per frame
KP_fw = Cm.Ki / fs / C2D / PX_PER_DEG_V1;     % counts per px per tick
KD_fw = Cm.Kp / C2D / PX_PER_DEG_V1;          % counts per px of frame-to-frame error change
fprintf('measured delay %.0f ms -> PI Kp %.3f, Ki %.2f 1/s, crossover %.2f Hz\n', ...
        1e3 * Tv_meas, Cm.Kp, Cm.Ki, infom.CrossoverFrequency / (2*pi));
fprintf('v1 firmware equivalent: KP_PAN = %.4f, KD_PAN = %.4f (current 0.0025 / 0.015)\n', KP_fw, KD_fw);
fprintf('  (KD maps to a proportional term only approximately: the firmware low-passes and decays it)\n');
Ki_v1 = 0.0025 * fs * C2D * PX_PER_DEG_V1;
fprintf('current v1 gains = PI with Ki %.2f 1/s, Kp %.3f -> crossover about %.2f Hz\n', ...
        Ki_v1, 0.015 * C2D * PX_PER_DEG_V1, Ki_v1 * abs(dcgain(G)) / (2*pi));

figure('Name', 'Part A');
yyaxis left;  plot(A.Tv_ms, A.wc_Hz, 'o-'); ylabel('achievable crossover [Hz] (PM 60^\circ)');
yyaxis right; plot(A.Tv_ms, A.Ki, 's--'); ylabel('K_i [1/s]');
xlabel('vision delay [ms]'); grid on; xline(1e3 * Tv_meas, ':', 'measured');
title('feedback loop: delay limits bandwidth');

%% Part B: tracking error, feedback (C0) vs prediction (C2), on hand-like targets
Gd = absorbDelay(ss(c2d(G, Ts, 'zoh')));
targets = make_targets(Ts);
Nf = round(fs / f_vision);               % ticks per vision frame
rng(1);

res = [];
for it = 1:numel(targets)
    r = targets(it).r;
    for iv = 1:numel(Tv_list)
        Tv = Tv_list(iv);
        [C0, ~] = pidtune(G * exp(-s * (Tv + Tzoh)), 'PI', optsPI);
        e0 = sim_loop(Gd, r, Ts, Nf, Tv, sigma_los, sigma_acc, 'C0', C0);
        e2 = sim_loop(Gd, r, Ts, Nf, Tv, sigma_los, sigma_acc, 'C2', totaldelay(G));
        res = [res; struct('target', targets(it).name, 'Tv_ms', 1e3 * Tv, ...
               'rms_C0', rms(e0.err), 'rms_C2', rms(e2.err), ...
               'max_C0', max(abs(e0.err)), 'max_C2', max(abs(e2.err)), ...
               'acc_C0', rms(e0.goal_acc), 'acc_C2', rms(e2.goal_acc))]; %#ok<AGROW>
    end
end
R = struct2table(res);
disp('=== Part B: tracking error [deg] and goal acceleration RMS [deg/s^2] ==='); disp(R);

figure('Name', 'Part B');
names = unique(R.target, 'stable');
for it = 1:numel(names)
    k = strcmp(R.target, names{it});
    subplot(1, numel(names), it);
    plot(R.Tv_ms(k), R.rms_C0(k), 'o-', R.Tv_ms(k), R.rms_C2(k), 's-'); grid on;
    xlabel('vision delay [ms]'); ylabel('RMS error [deg]'); title(names{it});
    legend('C0 feedback', 'C2 prediction', 'Location', 'northwest');
    xline(1e3 * Tv_meas, ':');
end
save(['loop_design_' AXIS '.mat'], 'A', 'R', 'Cm', 'G');

%% ------------------------------------------------------------------------------------------
function T = totaldelay(sys)
    T = sys.IODelay + sys.InputDelay + sys.OutputDelay;
end

function targets = make_targets(Ts)
    t = (0:Ts:20)';
    targets(1).name = 'ramp 34 deg/s';           % run 2 median hand speed
    targets(1).r = 34 * mod(t, 4) - 68;          % saw: 4 s sweeps of 136 deg
    targets(2).name = 'sine 0.5 Hz 20 deg';
    targets(2).r = 20 * sin(2*pi*0.5*t);
    targets(3).name = 'hand-like (band-limited)';
    rng(7);
    w = randn(size(t));
    [b, a] = butter(2, 1.5 * 2 * Ts);            % 1.5 Hz band limit
    x = filter(b, a, w);
    targets(3).r = 25 * x / std(x);
    f = fullfile('..', '..', 'docs', 'field_runs', 'run2', 'run2_los.csv');
    if isfile(f)                                  % real hand motion from field run 2, if exported
        L = readtable(f);
        targets(4).name = 'field run 2 LOS';
        targets(4).r = interp1(L.t_s - L.t_s(1), L.psi_deg, (0:Ts:L.t_s(end) - L.t_s(1))', 'linear');
    end
end

function out = sim_loop(Gd, r, Ts, Nf, Tv, sigma, sigma_acc, mode, arg)
    % Discrete simulation at the STM32 rate.
    %   vision: samples target and servo at frame instants, result arrives Tv later
    %   C0: error feedback e = r - y (pixel error), PI on the held error
    %   C2: absolute LOS r, CV Kalman filter at the frame rate, goal = prediction over
    %       (time since frame + Tv + servo dead time)
    N = numel(r);
    nd = round(Tv / Ts);
    x = zeros(size(Gd.A, 1), 1);
    y = zeros(N, 1); goal = zeros(N, 1);
    held = 0; integ = 0;
    % CV Kalman state for C2
    xs = [r(1); 0]; P = diag([1, 100]); dt = Nf * Ts;
    F = [1 dt; 0 1]; Q = sigma_acc^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2]; H = [1 0]; Rm = sigma^2;
    t_meas = -inf; xs_meas = xs;
    for k = 1:N
        % vision result for the frame captured nd ticks ago becomes available now
        kc = k - nd;
        if kc >= 1 && mod(kc - 1, Nf) == 0
            if strcmp(mode, 'C0')
                held = (r(kc) - y(kc)) + sigma * randn;
            else
                z = r(kc) + sigma * randn;
                xs = F * xs; P = F * P * F' + Q;
                K = P * H' / (H * P * H' + Rm);
                xs = xs + K * (z - H * xs); P = (eye(2) - K * H) * P;
                xs_meas = xs; t_meas = (kc - 1) * Ts;
            end
        end
        if strcmp(mode, 'C0')
            C = arg;
            integ = integ + C.Ki * held * Ts;
            goal(k) = integ + C.Kp * held;
        else
            lead = ((k - 1) * Ts - t_meas) + arg;       % since capture + servo dead time
            if isinf(lead), goal(k) = 0; else, goal(k) = xs_meas(1) + xs_meas(2) * lead; end
        end
        y(k) = Gd.C * x + Gd.D * goal(k);
        x = Gd.A * x + Gd.B * goal(k);
    end
    skip = round(1 / Ts);                                % ignore the first second
    out.err = r(skip:end) - y(skip:end);
    out.goal_acc = diff(goal(skip:end), 2) / Ts^2;
end
