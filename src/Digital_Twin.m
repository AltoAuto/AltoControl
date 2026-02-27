%% Lab 5 - System Identification Batch Processor
clear; close all; clc;

% Define the Sweep Parameters 
test_amps  = [0.5, 1.0];
test_freqs = [2.5, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, ...
              40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0];

% High-pass Butterworth filter coefficients for Digital Twin
a = [1.000000000000000, -1.991114292201654, 0.991153595868935];
b = [0.995566972017647, -1.991133944035295, 0.995566972017647];

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
        fname = fullfile('test_run', sprintf('sweep_%.1fV_%.1fHz.txt', Amp_in, Freq_in));
        
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

%% Algorithmic Parameter Extraction 
fprintf('\n--- Running Non-Linear Optimization for System ID ---\n');

% 1. Define the theoretical Magnitude equation (in dB)
mag_theory_db = @(Km,Tm,w) 20*log10( Km ./ (w .* sqrt(1 + (w.*Tm).^2)) );

% 2. Define the Cost Function (Mean Squared Error)
cost_function = @(q) mean(( mag_theory_db(exp(q(1)), exp(q(2)), omega_rad_sec) - Mag_dB_05V ).^2);

% 3. Provide an Initial Guess [Km_guess, Tm_guess]
initial_guess = log([100, 0.5]);

% 4. Run the Optimization Algorithm
options = optimset('Display','off','TolX',1e-8,'TolFun',1e-8);
optimal_params = fminsearch(cost_function, initial_guess, options);

% 5. Extract the mathematically perfect parameters
Km_opt = exp(optimal_params(1));
Tm_opt = exp(optimal_params(2));
final_mse = cost_function(optimal_params);

% 6. Console Output
fprintf('Optimal Parameters Found: Km = %.2f, Tm = %.4f with an MSE of %.2f\n', ...
        Km_opt, Tm_opt, final_mse);

%% Digital Twin Validation 
omega_smooth = logspace(log10(min(omega_rad_sec)), log10(max(omega_rad_sec)), 500);
Mag_dB_smooth = mag_theory_db(Km_opt, Tm_opt, omega_smooth);

% Overlay the optimized theoretical curve onto your existing experimental Bode plot
hold on;
plot(omega_smooth, Mag_dB_smooth, 'k--', 'LineWidth', 2, 'DisplayName', 'Optimized Digital Twin');
legend('Location', 'southwest');


%% Digital Twin Time-Domain Validation
fprintf('\n--- Generating Digital Twin Validation ---\n');

% 1. Build the Digital Twin (The Transfer Function)
% The manual defines H(s) = Km / (s * (Tm*s + 1)) which expands to:
% H(s) = Km / (Tm*s^2 + s + 0)
numerator = Km_opt;
denominator = [Tm_opt, 1, 0];
sys_motor = tf(numerator, denominator);

% 2. Generate the Virtual Input Signal
f_test = 2.5;                % Hz
Amp_test = 0.5;              % Volts
t_sim = 0 : (1/Fs) : 10;     % 10 seconds of simulated time at 1000 Hz
u_sim = Amp_test * sin(2 * pi * f_test * t_sim); % Virtual voltage command

% 3. Run the Virtual Simulation
[y_sim_raw, t_sim] = lsim(sys_motor, u_sim, t_sim);

% 4. Filter the Virtual Data (The Apples-to-Apples Step)
y_sim_filt = filter(b, a, y_sim_raw);

% 5. Load the Actual Physical Hardware Data for Comparison
fname_val = fullfile('test_run', sprintf('sweep_%.1fV_%.1fHz.txt', Amp_test, f_test));

raw_val_data = readmatrix(fname_val);
filt_hardware = raw_val_data(2:end, 2); % Extract the hardware's filtered column
t_hardware = (0:length(filt_hardware)-1) / Fs;

% 6. Align the Steady-State Data (Skip the transient)
idx0 = floor(Tdelay * Fs) + 1; % Skip the first second of filter transient

t_plot = t_hardware(idx0:end);
hardware_plot = filt_hardware(idx0:end);
sim_plot = y_sim_filt(idx0:length(t_hardware)); % Truncate sim to match hardware length

% 7. The Money Shot Plot
figure('Name', 'Digital Twin Validation', 'Color', 'w');
plot(t_plot, hardware_plot, 'r', 'LineWidth', 2, 'DisplayName', 'Physical Hardware Data');
hold on;
plot(t_plot, sim_plot, 'k--', 'LineWidth', 2, 'DisplayName', 'Digital Twin Simulation');

% Format the Plot
grid on;
xlim([Tdelay, Tdelay + 2]);
xlabel('Time (seconds)', 'FontWeight', 'bold');
ylabel('Motor Position (Encoder Counts)', 'FontWeight', 'bold');
title(sprintf('Digital Twin Validation: %.1f V at %.1f Hz', Amp_test, f_test), 'FontWeight', 'bold');
legend('Location', 'northeast');