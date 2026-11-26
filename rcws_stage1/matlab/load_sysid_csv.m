function d = load_sysid_csv(path)
%LOAD_SYSID_CSV  Read a sysid_capture.py CSV and put it on the 500 Hz tick grid.
%
%   d = load_sysid_csv('pan_prbs.csv')
%
%   d.t     tick times [s], uniform 2 ms
%   d.u     goal position relative to centre [deg]
%   d.y     measured position relative to centre [deg], resampled to the tick times
%   d.v     measured speed [deg/s]
%   d.load  load [%]
%   d.Ts    0.002
%   d.lat   per-sample latency: requesting tick -> status packet received [s]
%   d.raw   the table as read
%
%   The record of tick k carries the goal written in tick k and the latest status packet,
%   which was requested in tick k-1. y is therefore interpolated from its own receive time
%   (t_meas_s) onto the tick grid, so that the bus/reply delay ends up in the identified
%   dead time instead of being hidden by the record alignment.

C2D = 360 / 4096;                        % counts -> deg
T = readtable(path);
if height(T) < 100
    error('load_sysid_csv:short', '%s: only %d records', path, height(T));
end
if ismember('abort', T.Properties.VariableNames) && any(T.abort)
    error('load_sysid_csv:aborted', '%s: run was aborted by the firmware (stall watch or B1 ESTOP) - repeat it', path);
end

Ts = 0.002;
t = T.t_s - T.t_s(1);
% nominal grid (records are one per tick; small timer jitter is ignored)
d.t  = (0:height(T)-1)' * Ts;

fresh = T.fresh == 1 & T.rx_err == 0;
tm = T.t_meas_s(fresh) - T.t_s(1);
[tm, ia] = unique(tm);
yv = double(T.y(fresh));  yv = yv(ia);
vv = double(T.v(fresh));  vv = vv(ia);
lv = double(T.load(fresh)); lv = lv(ia);

d.u    = (double(T.u) - 2048) * C2D;
d.y    = (interp1(tm, yv, t, 'linear', 'extrap') - 2048) * C2D;
d.v    = fillmissing(interp1(tm, vv, t, 'previous'), 'nearest') * C2D;
d.load = fillmissing(interp1(tm, lv, t, 'previous'), 'nearest') * 0.1;
d.Ts   = Ts;
% the status packet in record k answers the READ sent in tick k-1
d.lat  = T.t_meas_s(fresh) - (T.t_s(fresh) - Ts);
d.raw  = T;

lost = 1 - mean(fresh);
fprintf('%s: %d samples, %.1f s, missing replies %.2f %%, reply latency p50 %.2f ms / p99 %.2f ms\n', ...
    path, height(T), d.t(end), 100 * lost, 1e3 * median(d.lat), 1e3 * prctile(d.lat, 99));
if max(abs(diff(t) - Ts)) > 0.5e-3
    warning('load_sysid_csv:jitter', 'tick jitter up to %.2f ms', 1e3 * max(abs(diff(t) - Ts)));
end
end
