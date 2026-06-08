function [t, TLTf, TLTr, Total_LLT, roll_angle, front_roll_out, rear_roll_out] = computeVehicleLLT(tspan, x0)

    %% ── Load Parameters & MoTec Data ────────────────────────────────────────────
    paramR26;
    load('MoTeC data/JTCendurance_test9.mat');

    %% ── Extract Signals ──────────────────────────────────────────────────────────
    % Lateral acceleration
    lat_g_time = Average_Lat_G.Time;
    lat_g_vec  = Average_Lat_G.Value;

    % Vehicle speed [m/s] — convert from km/h
    spd_time   = Vehicle_Speed_Value.Time;
    spd_vec    = Vehicle_Speed_Value.Value * 0.277778;

    % ── Linear Pod Displacements [m] ─────────────────────────────────────────────
    pod_time   = Damper_Front_Left_Linear.Time;

    disp_FL    = Damper_Front_Left_Linear.Value  * 1e-3;
    disp_FR    = Damper_Front_Right_Linear.Value * 1e-3;
    disp_RL    = Damper_Rear_Left_Linear.Value   * 1e-3;
    disp_RR    = Damper_Rear_Right_Linear.Value  * 1e-3;

    % ── Smooth Pod Data using Butterworth Filter ──────────────────────────────────
    fs = 1 / mean(diff(pod_time));
    fc = 5;
    [b, a] = butter(2, fc / (fs / 2));

    disp_FL_smooth = filtfilt(b, a, disp_FL);
    disp_FR_smooth = filtfilt(b, a, disp_FR);
    disp_RL_smooth = filtfilt(b, a, disp_RL);
    disp_RR_smooth = filtfilt(b, a, disp_RR);

    % ── Compute Pod Velocities ────────────────────────────────────────────────────
    dt_pod  = mean(diff(pod_time));
    vel_FL  = gradient(disp_FL_smooth, dt_pod);
    vel_FR  = gradient(disp_FR_smooth, dt_pod);
    vel_RL  = gradient(disp_RL_smooth, dt_pod);
    vel_RR  = gradient(disp_RR_smooth, dt_pod);

    % ── Compute Front & Rear Roll Angle and Rate from Pods ────────────────────────
    front_roll_meas      = (disp_FL_smooth - disp_FR_smooth) / car.track;
    front_roll_rate_meas = (vel_FL - vel_FR) / car.track;
    rear_roll_meas       = (disp_RL_smooth - disp_RR_smooth) / car.track;
    rear_roll_rate_meas  = (vel_RL - vel_RR) / car.track;

    %% ── Compute Axle Delay (Time-Varying) ────────────────────────────────────────
    v_min       = 1.0;
    spd_clamped = max(spd_vec, v_min);
    spd_on_pod  = interp1(spd_time, spd_clamped, pod_time, 'linear', 'extrap');
    tau_vec     = car.wheelbase ./ spd_on_pod;

    t_rear_delayed         = pod_time - tau_vec;
    t_rear_delayed_clamped = max(t_rear_delayed, pod_time(1));

    rear_roll_delayed      = interp1(pod_time, rear_roll_meas,      t_rear_delayed_clamped, 'linear', 'extrap');
    rear_roll_rate_delayed = interp1(pod_time, rear_roll_rate_meas, t_rear_delayed_clamped, 'linear', 'extrap');

    %% ── Default Inputs ───────────────────────────────────────────────────────────
    if nargin < 1 || isempty(tspan)
        tspan = [pod_time(1), pod_time(end)];
    end
    if nargin < 2 || isempty(x0)
        x0 = [
            interp1(pod_time, front_roll_meas,       tspan(1), 'linear', 'extrap');
            interp1(pod_time, front_roll_rate_meas,  tspan(1), 'linear', 'extrap');
            interp1(pod_time, rear_roll_delayed,     tspan(1), 'linear', 'extrap');
            interp1(pod_time, rear_roll_rate_delayed,tspan(1), 'linear', 'extrap');
        ];
    end

    %% ── Solve ODE ────────────────────────────────────────────────────────────────
    ode_fun = @(t_now, x) eom_local(t_now, x, ...
        lat_g_time, lat_g_vec, pod_time, ...
        front_roll_meas, front_roll_rate_meas, ...
        rear_roll_delayed, rear_roll_rate_delayed, ...
        car, front, rear, frontunsprung, rearunsprung);

    opts   = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);
    [t, x] = ode45(ode_fun, tspan, x0, opts);

    %% ── Post-Processing ──────────────────────────────────────────────────────────
    lat_g_out           = interp1(lat_g_time, lat_g_vec,           t, 'linear', 'extrap');
    front_roll_out      = interp1(pod_time,   front_roll_meas,     t, 'linear', 'extrap');
    front_roll_rate_out = interp1(pod_time,   front_roll_rate_meas,t, 'linear', 'extrap');
    rear_roll_out       = interp1(pod_time,   rear_roll_delayed,   t, 'linear', 'extrap');
    rear_roll_rate_out  = interp1(pod_time,   rear_roll_rate_delayed, t, 'linear', 'extrap');

    %% ── Geometric Load Transfer ──────────────────────────────────────────────────
    g_val    = 9.81;
    wd_front = weight_distribution;

    GLTf = (car.m * wd_front       * g_val .* lat_g_out * front.RC) / car.track;
    GLTr = (car.m * (1 - wd_front) * g_val .* lat_g_out * rear.RC)  / car.track;

    %% ── Elastic Load Transfer ────────────────────────────────────────────────────
    ELTf_spring = front_roll_out     .* front.k_roll;
    ELTf_damper = front_roll_rate_out .* front.cs_roll;
    ELTr_spring = rear_roll_out      .* rear.k_roll;
    ELTr_damper = rear_roll_rate_out  .* rear.cs_roll;

    %% ── Total Load Transfer ──────────────────────────────────────────────────────
    TLTf      = ELTf_spring + ELTf_damper + GLTf;
    TLTr      = ELTr_spring + ELTr_damper + GLTr;
    Total_LLT = (lat_g_out .* g_val .* car.m .* car.cgh) / car.track;
    roll_angle = (front_roll_out + rear_roll_out) / 2;

    if isempty(t) || isempty(TLTf) || isempty(TLTr) || isempty(Total_LLT) || isempty(roll_angle)
        error('Output variables are empty: check input data and calculations.');
    end

end


%% ── LOCAL ODE FUNCTION ───────────────────────────────────────────────────────
function dxdt = eom_local(t_now, x, ...
    lat_g_time, lat_g_vec, pod_time, ...
    front_roll_meas, front_roll_rate_meas, ...
    rear_roll_delayed, rear_roll_rate_delayed, ...
    car, front, rear, frontunsprung, rearunsprung)

    lat_g_t           = interp1(lat_g_time, lat_g_vec,           t_now, 'linear', 'extrap');
    front_roll_t      = interp1(pod_time,   front_roll_meas,      t_now, 'linear', 'extrap');
    front_roll_rate_t = interp1(pod_time,   front_roll_rate_meas, t_now, 'linear', 'extrap');
    rear_roll_t       = interp1(pod_time,   rear_roll_delayed,    t_now, 'linear', 'extrap');
    rear_roll_rate_t  = interp1(pod_time,   rear_roll_rate_delayed,t_now,'linear', 'extrap');

    [~, M] = roll_angle(lat_g_t, car, front, rear, frontunsprung, rearunsprung);

    kf = front.k_roll;  kr = rear.k_roll;
    cf = front.cs_roll; cr = rear.cs_roll;
    I  = 47.4;
    alpha = 50;

    err_front_disp = front_roll_t      - x(1);
    err_front_vel  = front_roll_rate_t - x(2);
    err_rear_disp  = rear_roll_t       - x(3);
    err_rear_vel   = rear_roll_rate_t  - x(4);

    dxdt    = zeros(4, 1);
    dxdt(1) = x(2) + alpha * err_front_disp;
    dxdt(2) = -(kf/I)*x(1) - (cf/I)*x(2) + M(1)/I + alpha * err_front_vel;
    dxdt(3) = x(4) + alpha * err_rear_disp;
    dxdt(4) = -(kr/I)*x(3) - (cr/I)*x(4) + M(2)/I + alpha * err_rear_vel;
end