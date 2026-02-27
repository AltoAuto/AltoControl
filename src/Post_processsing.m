%% Lab 5 - System Identification Batch Processor
clear; close all; clc;

% Define the Sweep Parameters 
test_amps  = [0.5, 1.0];
test_freqs = [2.5, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, ...
              40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0];

Fs = 1000;           % Hz (Sampling Frequency)
Tdelay = 1.071;        % Seconds to skip for filter transient

% Pre-allocate arrays to hold the final dB values for plotting
Mag_dB_05V = zeros(1, length(test_freqs));
Mag_dB_10V = zeros(1, length(test_freqs));

fprintf('Starting Batch Process...\n');

%% Process Every File Automatically
for a_idx = 1:length(test_amps)
    Amp_in = test_amps(a_idx);
    
    for f_idx = 1:length(test_freqs)
        Freq_in = test_freqs(f_idx);
        
        % Construct the filename
        fname = fullfile('Test_run', sprintf('sweep_%.1fV_%.1fHz.txt', Amp_in, Freq_in));
        
        % Safety check
        if ~isfile(fname)
            warning('File %s not found! Skipping...', fname);
            continue;
        end
        
        % Read the data
        % Row 1 [Freq, Amp]. Row 2 to End is [Raw, Filtered]
        raw_data = readmatrix(fname);
        filt = raw_data(2:end, 2); % Grab all rows from row 2 downward, column 2
        
        % Slice the steady-state segment
        idx0 = floor(Tdelay * Fs) + 1;
        if idx0 > length(filt)
             warning('File %s is too short! Skipping...', fname);
             continue;
        end
        fseg = filt(idx0:end);
        
        % Mean subtraction and RMS calculation
        fseg0 = fseg - mean(fseg);
        Crms = sqrt(mean(fseg0.^2));
        
        % Calculate Amplitudes
        Camp = sqrt(2) * Crms;       % Output Peak Amplitude
        Out_PP = 2 * Camp;           % Output Peak-to-Peak (Requested by manual Table 1 & 2)
        In_PP  = 2 * Amp_in;         % Input Peak-to-Peak (Requested by manual Table 1 & 2)
        
        % Calculate Gain and dB
        Gain = Out_PP / In_PP;       % Ratio of output to input 
        Gain_dB = 20 * log10(Gain);  % Convert to dB
        
        % Store the result in the correct array for plotting
        if Amp_in == 0.5
            Mag_dB_05V(f_idx) = Gain_dB;
        elseif Amp_in == 1.0
            Mag_dB_10V(f_idx) = Gain_dB;
        end
        
        fprintf('Processed %s -> Gain: %.2f dB\n', fname, Gain_dB);
    end
end

%% 3. Generate the Bode Plot
% Convert frequencies from Hz to rad/sec for the X-axis
omega_rad_sec = test_freqs * 2 * pi;

figure('Name', 'DC Servomotor System Identification', 'Color', 'w');
% Plot 0.5V Sweep
semilogx(omega_rad_sec, Mag_dB_05V, 'bo-', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', '0.5V Amplitude Input');
hold on;

% Plot 1.0V Sweep
semilogx(omega_rad_sec, Mag_dB_10V, 'rs-', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', '1.0V Amplitude Input');

% Format the Plot
grid on;
xlabel('Frequency \omega (rad/sec)', 'FontWeight', 'bold');
ylabel('Magnitude 20*log_{10}(Output/Input) (dB)', 'FontWeight', 'bold');
title('Open Loop Bode Plot of DC Servomotor', 'FontWeight', 'bold');
legend('Location', 'southwest');

% Set X-limits to make it easy to read at omega = 1
xlim([1, 1000]);
