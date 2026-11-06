%SERVO_SYSID  Identify the STS3215 goal-position -> position dynamics from sysid captures.
%
%   Inputs (from tools/sysid_capture.py, same folder or set DATA_DIR):
%     <axis>_chirp.csv   estimation data   (log chirp 0.2-12 Hz, +-5.3 deg)
%     <axis>_prbs.csv    validation data   (PRBS 20 ms bit, +-5.3 deg)
%     <axis>_step.csv    large-signal check (+-2.2 / 4.4 / 8.8 / 17.6 / 35 deg steps)
%
%   Output: servo_model_<axis>.mat with
%     G        chosen continuous model (tf with IODelay), deg -> deg
%     cand     all candidate models and their validation fits
%     steps    per-amplitude rise time / overshoot / peak speed (rate limit check)
%     meta     axis, files, reply latency, date
%
%   Requires System Identification Toolbox and Control System Toolbox.

% settings: set these in the workspace before running to override, e.g.
%   AXIS = 'tilt'; DATA_DIR = '../tools/synth'; servo_sysid
if ~exist('AXIS', 'var'),       AXIS = 'pan';    end   % 'pan' or 'tilt'
if ~exist('DATA_DIR', 'var'),   DATA_DIR = '.';  end
if ~exist('SPEED_MODE', 'var'), SPEED_MODE = ''; end   % '' (tracking default speed) or '_max'

f_est  = fullfile(DATA_DIR, [AXIS '_chirp' SPEED_MODE '.csv']);
f_val  = fullfile(DATA_DIR, [AXIS '_prbs'  SPEED_MODE '.csv']);
f_step = fullfile(DATA_DIR, [AXIS '_step'  SPEED_MODE '.csv']);

%% 1. data
de = load_sysid_csv(f_est);
dv = load_sysid_csv(f_val);
Ts = de.Ts;

est = iddata(de.y, de.u, Ts, 'InputName', 'goal', 'OutputName', 'pos', ...
             'InputUnit', 'deg', 'OutputUnit', 'deg');
val = iddata(dv.y, dv.u, Ts, 'InputName', 'goal', 'OutputName', 'pos', ...
             'InputUnit', 'deg', 'OutputUnit', 'deg');
% drop the 0.5 s pre-roll and remove the static offset (centre calibration, gravity on tilt)
est = detrend(est(251:end), 0);
val = detrend(val(251:end), 0);

%% 2. non-parametric reference: empirical frequency response from the chirp
Gspa = spafdr(est, [], logspace(log10(2*pi*0.2), log10(2*pi*12), 80));

%% 3. candidate parametric models (all with a free dead time)
opt = tfestOptions('InitializeMethod', 'iv', 'EnforceStability', true);
cand = struct('name', {}, 'model', {}, 'fit', {});
cand(end+1) = struct('name', '1 pole + delay',  'model', tfest(est, 1, 0, NaN, opt), 'fit', NaN);
cand(end+1) = struct('name', '2 poles + delay', 'model', tfest(est, 2, 0, NaN, opt), 'fit', NaN);
cand(end+1) = struct('name', '2 poles 1 zero + delay', 'model', tfest(est, 2, 1, NaN, opt), 'fit', NaN);
cand(end+1) = struct('name', 'P2DU (underdamped)', 'model', procest(est, 'P2DU'), 'fit', NaN);

for i = 1:numel(cand)
    [~, fitv] = compare(val, cand(i).model);
    cand(i).fit = fitv;
end
[~, best] = max([cand.fit]);
% prefer the simpler model when it is within 2 % of the best validation fit
for i = 1:best - 1
    if cand(i).fit >= cand(best).fit - 2
        best = i;
        break;
    end
end
G = tf(cand(best).model);

fprintf('\n=== %s: candidate models (validation = PRBS, NRMSE fit) ===\n', AXIS);
for i = 1:numel(cand)
    m = cand(i).model;
    fprintf('  %-26s fit %5.1f %%   delay %5.1f ms\n', cand(i).name, cand(i).fit, 1e3 * totaldelay(tf(m)));
end
bw = bandwidth(G);
fprintf('chosen: %s\n', cand(best).name);
fprintf('  DC gain %.3f, dead time %.1f ms, -3 dB bandwidth %.1f Hz\n', ...
        dcgain(G), 1e3 * totaldelay(G), bw / (2*pi));
p = pole(G);
for k = 1:numel(p)
    fprintf('  pole %+.1f %+.1fi rad/s  (wn %.1f Hz, zeta %.2f)\n', real(p(k)), imag(p(k)), ...
            abs(p(k)) / (2*pi), -real(p(k)) / abs(p(k)));
end

%% 4. large-signal check: does the linear model hold for big steps? (speed / accel limits)
steps = [];
if isfile(f_step)
    ds = load_sysid_csv(f_step);
    u = ds.u; y = ds.y; t = ds.t;
    edges = find(abs(diff(u)) > 0.5) + 1;          % goal changes
    for k = 1:numel(edges)
        i0 = edges(k);
        if k < numel(edges), i1 = edges(k+1) - 1; else, i1 = numel(u); end
        du = u(i0) - u(i0-1);
        seg = (y(i0:i1) - y(i0-1)) / du;            % normalised response 0 -> 1
        ts = t(i0:i1) - t(i0);
        r10 = find(seg >= 0.1, 1); r90 = find(seg >= 0.9, 1);
        if isempty(r10) || isempty(r90), continue; end
        st.du_deg    = du;
        i02 = find(seg >= 0.02, 1);
        st.dead_ms   = 1e3 * ts(i02);
        st.rise_ms   = 1e3 * (ts(r90) - ts(r10));
        st.overshoot = 100 * max(0, max(seg) - 1);
        st.vmax_dps  = max(abs(ds.v(i0:i1)));
        % linear-model prediction for the same step
        [ym, tm] = step(G * du, ts(end));
        ym = interp1(tm, ym, ts, 'linear', 'extrap') / du;
        m10 = find(ym >= 0.1, 1); m90 = find(ym >= 0.9, 1);
        if isempty(m10) || isempty(m90), st.model_rise_ms = NaN;
        else, st.model_rise_ms = 1e3 * (ts(m90) - ts(m10)); end
        steps = [steps; st]; %#ok<AGROW>
    end
    fprintf('\n=== step series (large-signal) ===\n');
    fprintf('  step [deg]  dead [ms]  rise [ms]  model rise [ms]  overshoot [%%]  peak speed [deg/s]\n');
    for k = 1:numel(steps)
        s = steps(k);
        fprintf('  %+9.1f  %9.1f  %9.1f  %15.1f  %13.1f  %18.0f\n', s.du_deg, s.dead_ms, ...
                s.rise_ms, s.model_rise_ms, s.overshoot, s.vmax_dps);
    end
    fprintf(['  -> where rise time grows with the step size and peak speed stops growing, the\n' ...
             '     servo is speed-limited (goal speed / accel registers): keep tracking moves in the\n' ...
             '     linear range or add a rate limit to the outer-loop model.\n']);
end

%% 5. figures
figure('Name', ['validation (PRBS) ' AXIS]);
compare(val, cand.model);
figure('Name', ['frequency response ' AXIS]);
bode(Gspa, G, {2*pi*0.2, 2*pi*12}); legend('chirp (SPAFDR)', 'model'); grid on;
figure('Name', ['latency and steps ' AXIS]);
subplot(1,2,1); plot(de.lat * 1e3, '.'); ylabel('reply latency [ms]'); xlabel('sample'); grid on;
subplot(1,2,2);
if ~isempty(steps)
    plot([steps.du_deg], [steps.rise_ms], 'o', [steps.du_deg], [steps.model_rise_ms], 'x');
    xlabel('step [deg]'); ylabel('10-90 % rise [ms]'); legend('measured', 'linear model'); grid on;
end

meta = struct('axis', AXIS, 'files', {{f_est, f_val, f_step}}, ...
              'reply_latency_ms_p50', 1e3 * median(de.lat), 'date', datestr(now));
save(['servo_model_' AXIS SPEED_MODE '.mat'], 'G', 'cand', 'steps', 'meta', 'Gspa');
fprintf('\nsaved servo_model_%s%s.mat\n', AXIS, SPEED_MODE);

function T = totaldelay(sys)
    T = sys.IODelay + sys.InputDelay + sys.OutputDelay;
end
