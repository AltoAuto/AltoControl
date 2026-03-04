%% Lab 5 - System Identification Batch Processor
clear; close all; clc;

%% ── Parameters ───────────────────────────────────────────────────────────────
test_amps  = [0.5, 1.0];
test_freqs = [2.5, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, ...
              40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0];

Fs     = 1000;    % Sampling frequency (Hz)
Tdelay = 1.071;   % Seconds to skip for filter transient

% Estimated plant parameters
Km      = 12672;
Tm      = 0.013;
omega_m = 1 / Tm;  % Break frequency ~76.9 rad/s

%% ── Pre-allocate ─────────────────────────────────────────────────────────────
Mag_dB_05V    = zeros(1, length(test_freqs));
Mag_dB_10V    = zeros(1, length(test_freqs));
Mag_dB_05V_bw = zeros(1, length(test_freqs));
Mag_dB_10V_bw = zeros(1, length(test_freqs));

% Also store counts for Post-Lab 2 tables
Counts_05V    = zeros(1, length(test_freqs));
Counts_10V    = zeros(1, length(test_freqs));
Counts_05V_bw = zeros(1, length(test_freqs));
Counts_10V_bw = zeros(1, length(test_freqs));

%% ── Batch Process ────────────────────────────────────────────────────────────
fprintf('Starting Batch Process...\n');


for a_idx = 1:length(test_amps)
    Amp_in = test_amps(a_idx);
    
    for f_idx = 1:length(test_freqs)
        Freq_in = test_freqs(f_idx);
        
        % Construct filename
        fname = fullfile('Test_run', sprintf('sweep_%.1fV_%.1fHz.txt', Amp_in, Freq_in));
        
        % Safety check
        if ~isfile(fname)
            warning('File %s not found! Skipping...', fname);
            continue;
        end
        
        % Read data
        % Row 1: [Freq, Amp] metadata
        % Row 2 to end: [raw_counts, filtered_counts]
        raw_data = readmatrix(fname);
        raw  = raw_data(2:end, 1);  % Column 1: raw encoder counts
        filt = raw_data(2:end, 2);  % Column 2: real-time HP filtered counts
        
        % Steady-state segment start index
        idx0 = floor(Tdelay * Fs) + 1;
        if idx0 > length(filt)
            warning('File %s is too short! Skipping...', fname);
            continue;
        end
        
        % Input peak-to-peak voltage
        In_PP = 2 * Amp_in;

        % ── Series A: Real-time C++ filtered ──────────────────────────────
        fseg      = filt(idx0:end);
        fseg0     = fseg - mean(fseg);
        Crms_A    = sqrt(mean(fseg0.^2));
        Out_PP_A  = 2 * sqrt(2) * Crms_A;
        Gain_dB_A = 20 * log10(Out_PP_A / In_PP);

        % ── Series B: MATLAB Butterworth HP filter on raw counts ───────────
        Fc        = Freq_in / 2;                   % Cutoff = half test frequency
        [b, a]    = butter(2, Fc/(Fs/2), 'high');
        raw_bw    = filter(b, a, raw);
        bseg      = raw_bw(idx0:end);
        bseg0     = bseg - mean(bseg);
        Crms_B    = sqrt(mean(bseg0.^2));
        Out_PP_B  = 2 * sqrt(2) * Crms_B;
        Gain_dB_B = 20 * log10(Out_PP_B / In_PP);

        % ── Store results ──────────────────────────────────────────────────
        if Amp_in == 0.5
            Mag_dB_05V(f_idx)    = Gain_dB_A;
            Mag_dB_05V_bw(f_idx) = Gain_dB_B;
            Counts_05V(f_idx)    = Out_PP_A;
            Counts_05V_bw(f_idx) = Out_PP_B;
        elseif Amp_in == 1.0
            Mag_dB_10V(f_idx)    = Gain_dB_A;
            Mag_dB_10V_bw(f_idx) = Gain_dB_B;
            Counts_10V(f_idx)    = Out_PP_A;
            Counts_10V_bw(f_idx) = Out_PP_B;
        end
    end
end

%% ── Print Tables ──────────────────────────────────────────────────
fprintf('\n========== V_in = 0.5V ==========\n');
fprintf('%-12s %-15s %-15s %-15s %-15s\n', ...
    'Freq(Hz)', 'RT Counts', 'RT Mag(dB)', 'BW Counts', 'BW Mag(dB)');
fprintf('%s\n', repmat('-', 1, 72));
for i = 1:length(test_freqs)
    fprintf('%-12.1f %-15.1f %-15.2f %-15.1f %-15.2f\n', ...
        test_freqs(i), Counts_05V(i), Mag_dB_05V(i), Counts_05V_bw(i), Mag_dB_05V_bw(i));
end

fprintf('\n========== V_in = 1.0V ==========\n');
fprintf('%-12s %-15s %-15s %-15s %-15s\n', ...
    'Freq(Hz)', 'RT Counts', 'RT Mag(dB)', 'BW Counts', 'BW Mag(dB)');
fprintf('%s\n', repmat('-', 1, 72));
for i = 1:length(test_freqs)
    fprintf('%-12.1f %-15.1f %-15.2f %-15.1f %-15.2f\n', ...
        test_freqs(i), Counts_10V(i), Mag_dB_10V(i), Counts_10V_bw(i), Mag_dB_10V_bw(i));
end

%% ── Post-Lab 1: Bode Plot with Asymptotes ────────────────────────────────────
omega_rad_sec = test_freqs * 2 * pi;

% Asymptote lines
omega_low  = logspace(0, log10(omega_m), 100);
asym_low   = 20*log10(Km) - 20*log10(omega_low);

y_break    = 20*log10(Km) - 20*log10(omega_m);
omega_high = logspace(log10(omega_m), 3, 100);
asym_high  = y_break - 40*(log10(omega_high) - log10(omega_m));

figure('Name', 'Post-Lab 1 - DC Servomotor Bode Plot', 'Color', 'w');

% Data series
semilogx(omega_rad_sec, Mag_dB_05V,    'bo-',  'LineWidth', 1.5, 'MarkerSize', 7, ...
    'DisplayName', '0.5V – RT Filter (C++)');
hold on;
semilogx(omega_rad_sec, Mag_dB_10V,    'rs-',  'LineWidth', 1.5, 'MarkerSize', 7, ...
    'DisplayName', '1.0V – RT Filter (C++)');
semilogx(omega_rad_sec, Mag_dB_05V_bw, 'b^--', 'LineWidth', 1.5, 'MarkerSize', 7, ...
    'DisplayName', '0.5V – MATLAB BW Filter');
semilogx(omega_rad_sec, Mag_dB_10V_bw, 'rv--', 'LineWidth', 1.5, 'MarkerSize', 7, ...
    'DisplayName', '1.0V – MATLAB BW Filter');

% Asymptotes
semilogx(omega_low,  asym_low,  'k-',  'LineWidth', 2.0, 'DisplayName', 'Asymptote -20 dB/dec');
semilogx(omega_high, asym_high, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Asymptote -40 dB/dec');

% Km reading point at omega = 1
plot(1, 20*log10(Km), 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'yellow', ...
    'DisplayName', sprintf('\\omega=1: %.1f dB  →  K_m = %.0f', 20*log10(Km), Km));

% Break frequency point
plot(omega_m, y_break, 'g^', 'MarkerSize', 12, 'MarkerFaceColor', 'green', ...
    'DisplayName', sprintf('\\omega_m = %.1f rad/s  →  T_m = %.3f s', omega_m, Tm));

% Vertical reference lines
xline(1,       '-.',  '\omega = 1',       'LabelVerticalAlignment', 'bottom', ...
    'Color', [0.5 0 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(omega_m, '-.',  '\omega_m = 76.9',  'LabelVerticalAlignment', 'bottom', ...
    'Color', [0 0.6 0],   'LineWidth', 1.2, 'HandleVisibility', 'off');

% Transfer function text box
str = sprintf(['Estimated Transfer Functions:\n' ...
               '0.5V:  \\theta(s)/V(s) = %d / [s(%.3fs + 1)]\n' ...
               '1.0V:  \\theta(s)/V(s) = %d / [s(%.3fs + 1)]'], ...
               Km, Tm, Km, Tm);
annotation('textbox', [0.13 0.08 0.45 0.12], 'String', str, ...
    'FitBoxToText', 'on', 'BackgroundColor', 'white', ...
    'EdgeColor', 'black', 'FontSize', 9);

% Formatting
grid on;
xlabel('Frequency \omega (rad/sec)',                       'FontWeight', 'bold', 'FontSize', 12);
ylabel('Magnitude  20·log_{10}(Counts / Voltage)  (dB)',  'FontWeight', 'bold', 'FontSize', 12);
title('Post-Lab 1: Open Loop Bode Plot – DC Servomotor',  'FontWeight', 'bold', 'FontSize', 13);
legend('Location', 'southwest', 'FontSize', 9);
xlim([1, 1000]);
ylim([0, 100]);
