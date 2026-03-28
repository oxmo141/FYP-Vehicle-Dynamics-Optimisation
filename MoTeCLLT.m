%% LATERAL LOAD TRANSFER FROM MOTEC I2PRO DATA

clear;clc

[t, TLTf, TLTr, Total_LLT] = computeVehicleLLT();

%% PLOTTING

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

%% SIGNAL SMOOTHENING

% Apply a moving average filter to smooth the signals
windowSize = 30; % Define the window size for smoothing
smoothTLTf = smoothdata(TLTf, 'movmean', windowSize);
smoothTLTr = smoothdata(TLTr, 'movmean', windowSize);
smoothTotalLLT = smoothdata(Total_LLT, 'movmean', windowSize);

figure(2);
title('Smoothened data')
plot(t, smoothTLTf, 'r', 'LineWidth', 1.5)
hold on
plot(t, smoothTLTr, 'b', 'LineWidth', 1.5)
hold on 
plot(t, smoothTotalLLT,'g', 'LineWidth', 1.5)

xlabel('Time (s)')
ylabel('Load Transfer (N)')
legend('Total Lateral Load Transfer Front', ...
    'Total Lateral Load Transfer Rear', ...
    'Total Lateral Load Transfer Car')
grid on

figure(3);
title('Check')
plot(t, TLTf, 'r', 'LineWidth', 1.5)
hold on
plot(t, smoothTLTf, 'b', 'LineWidth', 1.5)

xlabel('Time (s)')
ylabel('Load Transfer (N)')
legend('Total Lateral Load Transfer Front Raw', ...
    'Total Lateral Load Transfer Front Smoothened')