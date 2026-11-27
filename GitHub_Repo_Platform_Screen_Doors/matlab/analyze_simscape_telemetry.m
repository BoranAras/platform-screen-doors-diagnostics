close all;
clc;

% =========================================================================
% 0. DATA EXTRACTION & UNIT CONVERSION (m/s^2 -> g)
% =========================================================================
if ~exist('out', 'var')
    error('ERROR: "out" variable not found in workspace! Please run the PSD / PAKS simulation model first.');
end

% Time vector extraction
if isprop(out, 'zaman_verisi') || isfield(out, 'zaman_verisi')
    t_raw = out.zaman_verisi;
elseif isprop(out, 'tout') || isfield(out, 'tout')
    t_raw = out.tout;
else
    error('Time vector could not be resolved. Please verify Simulink logging.');
end

% Helper function for signal parsing
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

% Right Door Telemetry (converted to g)
a_sag = squeeze(parse_sim_data(out.sim_acceleration)) / g_std;
p_sag = squeeze(parse_sim_data(out.sim_position_sag));

% Left Door Telemetry (converted to g)
a_sol = squeeze(parse_sim_data(out.sim_acceleration2)) / g_std;
p_sol = squeeze(parse_sim_data(out.sim_position_sol));

% Time Vector
if isa(t_raw, 'timeseries')
    t = squeeze(t_raw.Time);
else
    t = squeeze(t_raw);
end

% Reshape to column vectors
t = t(:); 
a_sag = a_sag(:); 
p_sag = p_sag(:); 
a_sol = a_sol(:); 
p_sol = p_sol(:);

% =========================================================================
% 1. RIGHT DOOR DYNAMIC ATTENUATION (Distance Modeling)
% =========================================================================
kapi_genisligi = 0.8;       
sensor_konumu = 2.0;        
kapi_sag_uc_koordinati = p_sag + kapi_genisligi;
mesafe_sag = sensor_konumu - kapi_sag_uc_koordinati; 
sag_katsayi_uzakta = 0.3; 
sag_katsayi_yakinda = 1.0; 

max_mesafe_sag = max(mesafe_sag); 
min_mesafe_sag = min(mesafe_sag);

if max_mesafe_sag == min_mesafe_sag
    dinamik_katsayi_sag = ones(size(mesafe_sag)) * sag_katsayi_yakinda;
else
    dinamik_katsayi_sag = sag_katsayi_uzakta + ((max_mesafe_sag - mesafe_sag) / (max_mesafe_sag - min_mesafe_sag)) * (sag_katsayi_yakinda - sag_katsayi_uzakta);
end

a_sag_dinamik = a_sag .* dinamik_katsayi_sag;

% =========================================================================
% 2. LEFT DOOR DYNAMIC ATTENUATION (Distance Modeling)
% =========================================================================
stroke_sol = abs(p_sol); 
sol_kapi_sag_uc = -stroke_sol;
mesafe_sol = sensor_konumu - sol_kapi_sag_uc; 

sol_katsayi_yakinda = 0.25; 
sol_katsayi_uzakta  = 0.10; 

max_mesafe_sol = max(mesafe_sol); 
min_mesafe_sol = min(mesafe_sol);

if max_mesafe_sol == min_mesafe_sol
    dinamik_katsayi_sol = ones(size(mesafe_sol)) * sol_katsayi_yakinda;
else
    dinamik_katsayi_sol = sol_katsayi_uzakta + ((max_mesafe_sol - mesafe_sol) / (max_mesafe_sol - min_mesafe_sol)) * (sol_katsayi_yakinda - sol_katsayi_uzakta);
end

a_sol_dinamik = a_sol .* dinamik_katsayi_sol;

% =========================================================================
% 3. PHYSICAL SENSOR SUPERPOSITION
% =========================================================================
a_toplam = a_sag_dinamik + a_sol_dinamik;

% =========================================================================
% 4. RMS ENVELOPE & FFT SPECTRAL CALCULATIONS
% =========================================================================
Ts = mean(diff(t)); 
Fs = 1/Ts;
window_length = max(round(0.05 * Fs), 1);

% Moving RMS calculations for individual doors
rms_sag = sqrt(movmean(a_sag_dinamik.^2, window_length));
rms_sol = sqrt(movmean(a_sol_dinamik.^2, window_length));

% Root-Sum-Square (RSS) combined RMS
rms_toplam = sqrt(rms_sag.^2 + rms_sol.^2);

% FFT Spectral Analysis
L_toplam = length(a_toplam); 

Y_sag = fft(a_sag_dinamik);
P2_sag = abs(Y_sag/L_toplam); 
P1_sag = P2_sag(1:floor(L_toplam/2)+1); 
P1_sag(2:end-1) = 2*P1_sag(2:end-1);

Y_sol = fft(a_sol_dinamik);
P2_sol = abs(Y_sol/length(a_sol_dinamik)); 
P1_sol = P2_sol(1:floor(length(a_sol_dinamik)/2)+1); 
P1_sol(2:end-1) = 2*P1_sol(2:end-1);

Y_toplam = fft(a_toplam);
P2_toplam = abs(Y_toplam/L_toplam); 
P1_toplam = P2_toplam(1:floor(L_toplam/2)+1); 
P1_toplam(2:end-1) = 2*P1_toplam(2:end-1);

r_freq = Fs*(0:(floor(L_toplam/2)))/L_toplam;

% =========================================================================
% 5. TABBED GRAPHICAL VISUALIZATION (ENGLISH ANNOTATIONS)
% =========================================================================
fig = figure('Name', 'PSD Digital Twin Vibration & Spectral Analysis (g-Unit)', 'NumberTitle', 'off', 'Position', [100 100 1000 800]);
tgroup = uitabgroup(fig);

% --- TAB 1: RIGHT DOOR ---
tab1 = uitab(tgroup, 'Title', 'Right Door Dynamics');
subplot(3,1,1, 'Parent', tab1); 
plot(t, mesafe_sag, 'b', 'LineWidth', 1.5); 
title('Right Door - Relative Sensor Distance (m)'); 
xlabel('Time (s)'); ylabel('Distance (m)'); grid on;

subplot(3,1,2, 'Parent', tab1); 
plot(t, a_sag_dinamik, 'g'); hold on; 
plot(t, rms_sag, 'r', 'LineWidth', 2); 
title('Right Door Dynamic Acceleration & RMS Envelope (g)'); 
xlabel('Time (s)'); ylabel('Acceleration (g)'); 
legend('Raw Vibration (g)','RMS Envelope (g)', 'Location', 'northeast'); grid on;

subplot(3,1,3, 'Parent', tab1); 
plot(r_freq, P1_sag, 'm', 'LineWidth', 1.5); 
title('Right Door Single-Sided Amplitude Spectrum (FFT)'); 
xlabel('Frequency (Hz)'); ylabel('Amplitude |P1(f)| (g)'); 
xlim([0 100]); grid on;

% --- TAB 2: LEFT DOOR ---
tab2 = uitab(tgroup, 'Title', 'Left Door Dynamics');
subplot(3,1,1, 'Parent', tab2); 
plot(t, mesafe_sol, 'm', 'LineWidth', 1.5); 
title('Left Door - Relative Sensor Distance (m)'); 
xlabel('Time (s)'); ylabel('Distance (m)'); grid on;

subplot(3,1,2, 'Parent', tab2); 
plot(t, a_sol_dinamik, 'c'); hold on; 
plot(t, rms_sol, 'r', 'LineWidth', 2); 
title('Left Door Dynamic Acceleration & RMS Envelope (g)'); 
xlabel('Time (s)'); ylabel('Acceleration (g)'); 
legend('Raw Vibration (g)','RMS Envelope (g)', 'Location', 'northeast'); grid on;

subplot(3,1,3, 'Parent', tab2); 
plot(r_freq, P1_sol, 'm', 'LineWidth', 1.5); 
title('Left Door Single-Sided Amplitude Spectrum (FFT)'); 
xlabel('Frequency (Hz)'); ylabel('Amplitude |P1(f)| (g)'); 
xlim([0 100]); grid on;

% --- TAB 3: COMBINED PHYSICAL SENSOR TELEMETRY ---
tab3 = uitab(tgroup, 'Title', 'Superimposed Sensor Telemetry');
subplot(2,1,1, 'Parent', tab3);
plot(t, a_toplam, 'Color', [0.45 0.45 0.45], 'DisplayName', 'Superimposed Raw Signal (g)'); 
hold on; 
plot(t, rms_toplam, 'r', 'LineWidth', 2.5, 'DisplayName', 'RSS Combined RMS Envelope (g)');
title('Unified Physical Telemetry at Accelerometer Location (Right + Left Superposition)');
xlabel('Time (s)'); ylabel('Acceleration (g)'); 
legend('Location', 'northeast'); grid on;

subplot(2,1,2, 'Parent', tab3);
plot(r_freq, P1_toplam, 'b', 'LineWidth', 1.5);
title('Global Frequency Signature (Combined FFT Spectrum)');
xlabel('Frequency (Hz)'); ylabel('Amplitude |P1(f)| (g)'); 
xlim([0 100]); grid on;