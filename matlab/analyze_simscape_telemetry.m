close all;
clc;

% =========================================================================
% 0. DATA EXTRACTION & UNIT CONVERSION (m/s^2 -> g)
% Note: Output field names (e.g., zaman_verisi, sim_position_sag) match the
%       To Workspace block names defined inside platform_screen_doors_multibody.slx
% =========================================================================
if ~exist('out', 'var')
    error('ERROR: "out" variable not found in workspace. Run the Simscape simulation model first.');
end

% Extract simulation time vector
if isprop(out, 'zaman_verisi') || isfield(out, 'zaman_verisi')
    t_raw = out.zaman_verisi;
elseif isprop(out, 'tout') || isfield(out, 'tout')
    t_raw = out.tout;
else
    error('Time vector could not be resolved. Verify Simulink output logging.');
end

% Helper function for Simulink signal parsing
function data = parse_sim_data(sig)
    if isa(sig, 'timeseries')
        data = sig.Data;
    elseif isstruct(sig) && isfield(sig, 'signals')
        data = sig.signals.values;
    else
        data = sig;
    end
end

% Standard gravitational acceleration (m/s^2 -> g conversion)
g_std = 9.81;

% Right door leaf telemetry (acceleration converted to g)
acc_right = squeeze(parse_sim_data(out.sim_acceleration)) / g_std;
pos_right = squeeze(parse_sim_data(out.sim_position_sag));

% Left door leaf telemetry (acceleration converted to g)
acc_left = squeeze(parse_sim_data(out.sim_acceleration2)) / g_std;
pos_left = squeeze(parse_sim_data(out.sim_position_sol));

% Time vector formatting
if isa(t_raw, 'timeseries')
    t = squeeze(t_raw.Time);
else
    t = squeeze(t_raw);
end

% Ensure column vectors
t = t(:);
acc_right = acc_right(:);
pos_right = pos_right(:);
acc_left = acc_left(:);
pos_left = pos_left(:);

% =========================================================================
% 1. RIGHT DOOR EXPLORATORY DISTANCE-DEPENDENT ATTENUATION
% =========================================================================
door_width = 0.8;          % Door leaf width (m)
sensor_position = 2.0;     % Reference accelerometer mounting location (m)

right_edge_coord = pos_right + door_width;
dist_right = sensor_position - right_edge_coord;

right_weight_far = 0.30;
right_weight_near = 1.00;

max_dist_right = max(dist_right);
min_dist_right = min(dist_right);

if max_dist_right == min_dist_right
    weight_right = ones(size(dist_right)) * right_weight_near;
else
    weight_right = right_weight_far + ...
        ((max_dist_right - dist_right) / (max_dist_right - min_dist_right)) * ...
        (right_weight_near - right_weight_far);
end

acc_right_weighted = acc_right .* weight_right;

% =========================================================================
% 2. LEFT DOOR EXPLORATORY DISTANCE-DEPENDENT ATTENUATION
% =========================================================================
left_stroke = abs(pos_left);
left_edge_coord = -left_stroke;
dist_left = sensor_position - left_edge_coord;

left_weight_near = 0.25;
left_weight_far  = 0.10;

max_dist_left = max(dist_left);
min_dist_left = min(dist_left);

if max_dist_left == min_dist_left
    weight_left = ones(size(dist_left)) * left_weight_near;
else
    weight_left = left_weight_far + ...
        ((max_dist_left - dist_left) / (max_dist_left - min_dist_left)) * ...
        (left_weight_near - left_weight_far);
end

acc_left_weighted = acc_left .* weight_left;

% =========================================================================
% 3. SUPERPOSED SIGNAL AT ACCELEROMETER LOCATION
% =========================================================================
acc_total = acc_right_weighted + acc_left_weighted;

% =========================================================================
% 4. MOVING RMS ENVELOPE & SINGLE-SIDED FFT SPECTRUM
% =========================================================================
dt_mean = mean(diff(t));
fs = 1 / dt_mean;
window_samples = max(round(0.05 * fs), 1);  % 50 ms moving RMS window

% Individual door moving RMS envelopes
rms_right = sqrt(movmean(acc_right_weighted.^2, window_samples));
rms_left  = sqrt(movmean(acc_left_weighted.^2, window_samples));

% Combined moving RMS (computed directly on superposed signal)
rms_total = sqrt(movmean(acc_total.^2, window_samples));

% Single-sided FFT amplitude spectra
n_total = length(acc_total);

fft_right = fft(acc_right_weighted);
p2_right = abs(fft_right / n_total);
p1_right = p2_right(1:floor(n_total/2)+1);
p1_right(2:end-1) = 2 * p1_right(2:end-1);

n_left = length(acc_left_weighted);
fft_left = fft(acc_left_weighted);
p2_left = abs(fft_left / n_left);
p1_left = p2_left(1:floor(n_left/2)+1);
p1_left(2:end-1) = 2 * p1_left(2:end-1);

fft_total = fft(acc_total);
p2_total = abs(fft_total / n_total);
p1_total = p2_total(1:floor(n_total/2)+1);
p1_total(2:end-1) = 2 * p1_total(2:end-1);

freq_axis = fs * (0:floor(n_total/2)) / n_total;

% =========================================================================
% 5. TABBED GRAPHICAL VISUALIZATION
% =========================================================================
fig = figure('Name', 'PSD Simulation Vibration & Spectral Analysis (g-Unit)', ...
             'NumberTitle', 'off', 'Position', [100 100 1000 800]);
tgroup = uitabgroup(fig);

% --- TAB 1: RIGHT DOOR ---
tab1 = uitab(tgroup, 'Title', 'Right Door Dynamics');
subplot(3,1,1, 'Parent', tab1);
plot(t, dist_right, 'b', 'LineWidth', 1.5);
title('Right Door - Relative Sensor Distance (m)');
xlabel('Time (s)'); ylabel('Distance (m)'); grid on;

subplot(3,1,2, 'Parent', tab1);
plot(t, acc_right_weighted, 'g'); hold on;
plot(t, rms_right, 'r', 'LineWidth', 2);
title('Right Door Dynamic Acceleration & Moving RMS Envelope (g)');
xlabel('Time (s)'); ylabel('Acceleration (g)');
legend('Simulated Vibration (g)', 'Moving RMS Envelope (g)', 'Location', 'northeast'); grid on;

subplot(3,1,3, 'Parent', tab1);
plot(freq_axis, p1_right, 'm', 'LineWidth', 1.5);
title('Right Door Single-Sided Amplitude Spectrum (FFT)');
xlabel('Frequency (Hz)'); ylabel('Amplitude |P1(f)| (g)');
xlim([0 100]); grid on;

% --- TAB 2: LEFT DOOR ---
tab2 = uitab(tgroup, 'Title', 'Left Door Dynamics');
subplot(3,1,1, 'Parent', tab2);
plot(t, dist_left, 'm', 'LineWidth', 1.5);
title('Left Door - Relative Sensor Distance (m)');
xlabel('Time (s)'); ylabel('Distance (m)'); grid on;

subplot(3,1,2, 'Parent', tab2);
plot(t, acc_left_weighted, 'c'); hold on;
plot(t, rms_left, 'r', 'LineWidth', 2);
title('Left Door Dynamic Acceleration & Moving RMS Envelope (g)');
xlabel('Time (s)'); ylabel('Acceleration (g)');
legend('Simulated Vibration (g)', 'Moving RMS Envelope (g)', 'Location', 'northeast'); grid on;

subplot(3,1,3, 'Parent', tab2);
plot(freq_axis, p1_left, 'm', 'LineWidth', 1.5);
title('Left Door Single-Sided Amplitude Spectrum (FFT)');
xlabel('Frequency (Hz)'); ylabel('Amplitude |P1(f)| (g)');
xlim([0 100]); grid on;

% --- TAB 3: COMBINED TELEMETRY ---
tab3 = uitab(tgroup, 'Title', 'Superposed Sensor Telemetry');
subplot(2,1,1, 'Parent', tab3);
plot(t, acc_total, 'Color', [0.45 0.45 0.45], 'DisplayName', 'Superposed Simulated Signal (g)');
hold on;
plot(t, rms_total, 'r', 'LineWidth', 2.5, 'DisplayName', 'Combined Moving RMS Envelope (g)');
title('Combined Simulated Telemetry at Accelerometer Location (Right + Left)');
xlabel('Time (s)'); ylabel('Acceleration (g)');
legend('Location', 'northeast'); grid on;

subplot(2,1,2, 'Parent', tab3);
plot(freq_axis, p1_total, 'b', 'LineWidth', 1.5);
title('Combined Frequency Spectrum (FFT)');
xlabel('Frequency (Hz)'); ylabel('Amplitude |P1(f)| (g)');
xlim([0 100]); grid on;
