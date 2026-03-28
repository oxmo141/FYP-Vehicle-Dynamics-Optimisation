%% LATERAL LOAD TRANSFER FROM MOTEC I2PRO DATA

clear;clc
paramR26

% Run param; first to load parameters
paramR26;

lat_g_values = 0.5:0.1:1.5;
tspan = [0 1];
x0 = [0 0 0 0];

for i = 1:length(lat_g_values)
    lat_g = lat_g_values(i);
    [t, TLTf, TLTr, Total_LLT] = computeVehicleLLT(lat_g, tspan, x0);

    % Example: Plot or analyze TLTf(end) for steady-state at t=1s
    fprintf('lat_g = %.1fg: Front TLT = %.2f N, Rear TLT = %.2f N\n', ...
        lat_g, TLTf(end), TLTr(end));
end
