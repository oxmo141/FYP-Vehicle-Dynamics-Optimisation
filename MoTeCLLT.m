%% LATERAL LOAD TRANSFER FROM MOTEC I2PRO DATA

clear;clc

[t, TLTf, TLTr, Total_LLT] = computeVehicleLLT();

%% PLOTTING
MoTecData;
paramR26;

figure(1);
title('Raw Data')
plot(t, TLTf, 'r', 'LineWidth', 1.5)
hold on
plot(t, TLTr, 'b', 'LineWidth', 1.5)
hold on 
plot(t, Total_LLT, 'g', 'LineWidth', 1.5)


xlabel('Time (s)')
ylabel('Load Transfer (N)')
legend('Total Lateral Load Transfer Front', ...
    'Total Lateral Load Transfer Rear', ...
    'Total Lateral Load Transfer Car')
grid on

%% CHECK TOTAL LOAD TRANSFER

% Assuming data.Average_Lateral_G.Time holds the time corresponding to the lateral G values
t_lateral = data.Average_Lateral_G.Time;  % Extract the time vector for lateral G
lateral_g_raw = data.Average_Lateral_G.Value;  % Extract the lateral G values

% Interpolate lateral G values onto the common time vector t
lateral_g_interp = interp1(t_lateral, lateral_g_raw, t, 'linear', 'extrap');

theoretical_total_load = (lateral_g_interp*9.81*car.m*car.cgh)/car.track;

figure(2);
plot(t, theoretical_total_load, 'r', 'LineWidth', 1.5)
hold on
plot(t, Total_LLT,'b' ,'LineWidth', 1.5)

xlabel('Time (s)')
ylabel('Load Transfer (N)')
legend('Total Car Baseline', ...
    'Total Lateral Load Transfer Car')
grid on


% %% SIGNAL SMOOTHENING
% 
% % Apply a moving average filter to smooth the signals
% windowSize = 30; % Define the window size for smoothing
% smoothTLTf = smoothdata(TLTf, 'movmean', windowSize);
% smoothTLTr = smoothdata(TLTr, 'movmean', windowSize);
% smoothTotalLLT = smoothdata(Total_LLT, 'movmean', windowSize);
% 
% figure(2);
% title('Smoothened data')
% plot(t, smoothTLTf, 'r', 'LineWidth', 1.5)
% hold on
% plot(t, smoothTLTr, 'b', 'LineWidth', 1.5)
% hold on 
% plot(t, smoothTotalLLT,'g', 'LineWidth', 1.5)
% 
% xlabel('Time (s)')
% ylabel('Load Transfer (N)')
% legend('Total Lateral Load Transfer Front', ...
%     'Total Lateral Load Transfer Rear', ...
%     'Total Lateral Load Transfer Car')
% grid on
% 
% figure(3);
% title('Check')
% plot(t, TLTf, 'r', 'LineWidth', 1.5)
% hold on
% plot(t, smoothTLTf, 'b', 'LineWidth', 1.5)
% 
% xlabel('Time (s)')
% ylabel('Load Transfer (N)')
% legend('Total Lateral Load Transfer Front Raw', ...
%     'Total Lateral Load Transfer Front Smoothened')