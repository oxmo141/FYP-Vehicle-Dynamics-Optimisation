%% YAW
clear; clc;

%% READ DATA
[t, TLTf, TLTr, Total_LLT] = computeVehicleLLT();
MoTecData;
paramR26;

%% EXTRACT RAW SIGNALS WITH THEIR OWN TIME VECTORS
t_yaw   = data.G_Sensor_Front_Yaw_Rate.Time;
t_delta = data.Steering_Angle.Time;
t_speed = data.Vehicle_Speed_Value.Time;

yaw_raw   = data.G_Sensor_Front_Yaw_Rate.Value * (pi/180);  % [rad/s]
delta_raw = data.Steering_Angle.Value           * (pi/180) / 3;  % [rad]
V_raw     = data.Vehicle_Speed_Value.Value * 0.277778;                        % [m/s]

%% DEFINE COMMON TIME VECTOR
% Use the overlapping range across all signals
t_start = max([t(1), t_yaw(1), t_delta(1), t_speed(1)]);
t_end   = min([t(end), t_yaw(end), t_delta(end), t_speed(end)]);
dt      = mean(diff(t));  % Use LLT time step as reference

t_common = (t_start : dt : t_end)';

%% INTERPOLATE ALL SIGNALS ONTO COMMON TIME VECTOR
yaw_v = interp1(t_yaw,   yaw_raw,   t_common, 'linear', 'extrap');
delta = interp1(t_delta, delta_raw, t_common, 'linear', 'extrap');
V     = interp1(t_speed, V_raw,     t_common, 'linear', 'extrap');

% Also interpolate LLT signals if needed later
TLTf_c      = interp1(t, TLTf,      t_common, 'linear', 'extrap');
TLTr_c      = interp1(t, TLTr,      t_common, 'linear', 'extrap');
Total_LLT_c = interp1(t, Total_LLT, t_common, 'linear', 'extrap');

%% VEHICLE PARAMETERS (from paramR26)
m = car.m;  % Vehicle mass [kg]
L = car.wheelbase;  % Wheelbase [m]

%% COMPUTE C_ALPHA
% Mask low-speed and near-zero steering to avoid division instability
speed_thresh = 5;   % [m/s] - ignore data below this speed
delta_thresh = 0.01; % [rad] - ignore near-zero steering (~0.57 deg)

valid = (V > speed_thresh) & (abs(delta) > delta_thresh);

C_alpha = zeros(size(t_common));
C_alpha(valid) = (m * L .* yaw_v(valid)) ./ (V(valid) .* delta(valid));

% Optional: Smooth C_alpha to reduce noise
windowSize = 50;
C_alpha_smooth = movmean(C_alpha(valid), windowSize);

%% PLOTTING
figure;

subplot(3,1,1);
plot(t_common, V, 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Vehicle Speed'); grid on;

subplot(3,1,2);
plot(t_common, delta * (180/pi), 'r', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Steering Angle [deg]');
title('Steering Angle'); grid on;

subplot(3,1,3);
plot(t_common(valid), C_alpha(valid), 'Color', [0.7 0.7 0.7], ...
    'LineWidth', 1, 'DisplayName', 'Raw');
hold on;
plot(t_common(valid), C_alpha_smooth, 'k', ...
    'LineWidth', 2, 'DisplayName', 'Smoothed');
xlabel('Time [s]'); ylabel('C_\alpha [N/rad]');
title('Estimated Tire Cornering Stiffness');
legend; grid on;

%% SUMMARY STATISTICS (valid regions only)
fprintf('--- C_alpha Estimates (valid regions) ---\n');
fprintf('Mean   C_alpha: %.2f N/rad\n', mean(C_alpha(valid)));
fprintf('Median C_alpha: %.2f N/rad\n', median(C_alpha(valid)));
fprintf('StdDev C_alpha: %.2f N/rad\n', std(C_alpha(valid)));
fprintf('Max    C_alpha: %.2f N/rad\n', max(C_alpha(valid)));
