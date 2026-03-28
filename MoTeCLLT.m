%% LATERAL LOAD TRANSFER FROM MOTEC I2PRO DATA

clear;clc

[t, TLTf, TLTr, Total_LLT] = computeVehicleLLT();

figure(1);
plot(t, TLTf, 'LineWidth', 1.5)
hold on
plot(t, TLTr, 'LineWidth', 1.5)
hold on 
plot(t, Total_LLT, 'LineWidth', 1.5)

xlabel('Time (s)')
ylabel('Load Transfer (N)')
legend('Total Lateral Load Transfer Front', ...
    'Total Lateral Load Transfer Rear', ...
    'Total Lateral Load Transfer Car')
grid on