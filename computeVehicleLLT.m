function [t, TLTf, TLTr, Total_LLT] = computeVehicleLLT(tspan, x0)
% COMPUTEVEHICLELLT Computes lateral load transfer (LLT) using time-varying
% lat_g, corner displacements from linear pods, and axle delay from vehicle speed.
%
% Inputs:
%   tspan    - [t_start t_end] (default: full MoTec time range)
%   x0       - [front_roll; front_roll_rate; rear_roll; rear_roll_rate] (default: zeros)
%
% Outputs:
%   t         - Time vector from ODE solver
%   TLTf      - Total lateral load transfer, front axle (Nx1)
%   TLTr      - Total lateral load transfer, rear axle (Nx1)
%   Total_LLT - Total vehicle LLT verification (Nx1)

%% ── Load Parameters & MoTec Data ────────────────────────────────────────────
paramR26;
MoTecData;

%% ── Extract Signals ──────────────────────────────────────────────────────────

% Lateral acceleration
lat_g_time = data.Average_Lateral__G_.Time;
lat_g_vec  = data.Average_Lateral__G_.Value;

% Vehicle speed (used for axle delay computation)
spd_time   = data.Vehicle_Speed_Value.Time;
spd_vec    = data.Vehicle_Speed_Value.Value * 0.277778;  % [m/s] — convert 

% ── Linear Pod Displacements [m] ─────────────────────────────────────────────
% Adjust field names to match your MoTec channel names exactly
pod_time   = data.Damper_Front_Left_Linear.Time;  % Assume all pods share same time base

disp_FL    = data.Damper_Front_Left_Linear.Value * 1E-3;
disp_FR    = data.Damper_Front_Right_Linear.Value * 1E-3;
disp_RL    = data.Damper_Rear_Left_Linear.Value * 1E-3;
disp_RR    = data.Damper_Rear_Right_Linear.Value * 1E-3;

% ── Compute Pod Velocities (central difference) ───────────────────────────────
dt_pod     = mean(diff(pod_time));          % Assumes uniform sampling
vel_FL     = gradient(disp_FL, dt_pod);
vel_FR     = gradient(disp_FR, dt_pod);
vel_RL     = gradient(disp_RL, dt_pod);
vel_RR     = gradient(disp_RR, dt_pod);

% ── Compute Front Roll Angle & Rate from Pods ─────────────────────────────────
% Roll angle  = (disp_FL - disp_FR) / track  [rad, small angle assumption]
% Roll rate   = (vel_FL  - vel_FR)  / track
front_roll_meas = (disp_FL - disp_FR) / car.track;
front_roll_rate_meas = (vel_FL  - vel_FR)  / car.track;

rear_roll_meas  = (disp_RL - disp_RR) / car.track;
rear_roll_rate_meas  = (vel_RL  - vel_RR)  / car.track;

%% ── Compute Axle Delay (Time-Varying) ────────────────────────────────────────
% Rear axle sees same road event as front axle after delay τ(t) = L / v(t)
% We compute a cumulative delay time-shift for the rear axle signals.
%
%   τ(t) = wheelbase / v(t)
%
% Clamp speed to avoid division by zero at standstill.

v_min      = 1.0;                          % Minimum speed threshold [m/s]
spd_clamped = max(spd_vec, v_min);

% Interpolate speed onto pod time base
spd_on_pod  = interp1(spd_time, spd_clamped, pod_time, 'linear', 'extrap');

% Time delay at each moment
tau_vec     = car.wheelbase ./ spd_on_pod; % [s], time-varying

% Build delayed rear time axis:
% For each time index i, the rear axle at time t(i) corresponds to
% what the front axle saw at time t(i) - tau(i).
t_rear_delayed = pod_time - tau_vec;       % Shifted time vector

% Interpolate rear pod signals onto delayed time axis
% (clamp to valid range to avoid extrapolation artefacts)
t_rear_delayed_clamped = max(t_rear_delayed, pod_time(1));

rear_roll_delayed      = interp1(pod_time, rear_roll_meas, ...
                                 t_rear_delayed_clamped, 'linear', 'extrap');
rear_roll_rate_delayed = interp1(pod_time, rear_roll_rate_meas, ...
                                 t_rear_delayed_clamped, 'linear', 'extrap');

%% ── Default Inputs ───────────────────────────────────────────────────────────
if nargin < 1 || isempty(tspan)
    tspan = [pod_time(1), pod_time(end)];
end
if nargin < 2 || isempty(x0)
    % Seed initial conditions from measured pod data at t_start
    x0 = [
        interp1(pod_time, front_roll_meas,       tspan(1), 'linear', 'extrap');
        interp1(pod_time, front_roll_rate_meas,  tspan(1), 'linear', 'extrap');
        interp1(pod_time, rear_roll_delayed,     tspan(1), 'linear', 'extrap');
        interp1(pod_time, rear_roll_rate_delayed,tspan(1), 'linear', 'extrap');
    ];
end

%% ── Anonymous Function Handle (Passes All Signals by Closure) ────────────────
ode_fun = @(t_now, x) eom_local(t_now, x, ...
    lat_g_time,  lat_g_vec,  ...
    pod_time,                ...
    front_roll_meas,         front_roll_rate_meas, ...
    rear_roll_delayed,       rear_roll_rate_delayed, ...
    car, front, rear, frontunsprung, rearunsprung);

%% ── Solve ODE ────────────────────────────────────────────────────────────────
opts = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);
[t, x] = ode45(ode_fun, tspan, x0, opts);

%% ── Post-Processing: Interpolate All Signals onto ODE Time Vector ────────────
lat_g_out         = interp1(lat_g_time, lat_g_vec,          t, 'linear', 'extrap');
front_roll_out    = interp1(pod_time,   front_roll_meas,     t, 'linear', 'extrap');
front_roll_rate_out = interp1(pod_time, front_roll_rate_meas,t, 'linear', 'extrap');
rear_roll_out     = interp1(pod_time,   rear_roll_delayed,   t, 'linear', 'extrap');
rear_roll_rate_out = interp1(pod_time,  rear_roll_rate_delayed, t, 'linear', 'extrap');

%% ── Geometric Load Transfer (Time-Varying lat_g) ─────────────────────────────
g_val    = 9.81;
wd_front = weight_distribution / 100;

GLTf = (car.m * wd_front       * g_val .* lat_g_out * front.RC) / car.track;
GLTr = (car.m * (1 - wd_front) * g_val .* lat_g_out * rear.RC)  / car.track;

%% ── Elastic Load Transfer (Using Measured Pod States) ────────────────────────
% Use measured displacements/velocities (from pods) for spring and damper LLT
ELTf_spring = front_roll_out     .* front.k_roll;
ELTf_damper = front_roll_rate_out .* front.cs_roll;
ELTr_spring = rear_roll_out      .* rear.k_roll;
ELTr_damper = rear_roll_rate_out  .* rear.cs_roll;

%% ── Total Load Transfer ──────────────────────────────────────────────────────
TLTf = ELTf_spring + ELTf_damper + GLTf;
TLTr = ELTr_spring + ELTr_damper + GLTr;

%% ── Total Vehicle LLT (Verification) ─────────────────────────────────────────
Total_LLT = (lat_g_out .* g_val .* car.m .* car.cgh) / car.track;

end % ── END computeVehicleLLT ──────────────────────────────────────────────────


%% ══ STANDALONE ODE FUNCTION ═══════════════════════════════════════════════════
function dxdt = eom_local(t_now, x, ...
    lat_g_time, lat_g_vec, ...
    pod_time, ...
    front_roll_meas, front_roll_rate_meas, ...
    rear_roll_delayed, rear_roll_rate_delayed, ...
    car, front, rear, frontunsprung, rearunsprung)

    % ── Interpolate lat_g at current time ────────────────────────────────────
    lat_g_t = interp1(lat_g_time, lat_g_vec, t_now, 'linear', 'extrap');

    % ── Interpolate measured pod states at current time ───────────────────────
    front_roll_t      = interp1(pod_time, front_roll_meas,      t_now, 'linear', 'extrap');
    front_roll_rate_t = interp1(pod_time, front_roll_rate_meas, t_now, 'linear', 'extrap');
    rear_roll_t       = interp1(pod_time, rear_roll_delayed,    t_now, 'linear', 'extrap');
    rear_roll_rate_t  = interp1(pod_time, rear_roll_rate_delayed,t_now,'linear', 'extrap');

    % ── Compute moments ───────────────────────────────────────────────────────
    [~, M] = roll_angle(lat_g_t, car, front, rear, frontunsprung, rearunsprung);

    % ── Roll stiffness & damping ──────────────────────────────────────────────
    kf = front.k_roll;
    kr = rear.k_roll;
    cf = front.cs_roll;
    cr = rear.cs_roll;
    I  = 182.24965; % Roll inertia [kg·m²]

    % ── Error between ODE state and measured pod state ────────────────────────
    % The ODE corrects itself toward measured displacement/velocity
    % This acts as a soft constraint — the ODE is guided by real data
    err_front_disp = front_roll_t      - x(1);
    err_front_vel  = front_roll_rate_t - x(2);
    err_rear_disp  = rear_roll_t       - x(3);
    err_rear_vel   = rear_roll_rate_t  - x(4);

    alpha = 50; % Correction gain — increase to track measured data more tightly

    dxdt = zeros(4, 1);

    % Front roll dynamics + measured correction
    dxdt(1) = x(2) + alpha * err_front_disp;
    dxdt(2) = -(kf / I) * x(1) - (cf / I) * x(2) + M(1) / I + alpha * err_front_vel;

    % Rear roll dynamics + measured correction (already delay-compensated)
    dxdt(3) = x(4) + alpha * err_rear_disp;
    dxdt(4) = -(kr / I) * x(3) - (cr / I) * x(4) + M(2) / I + alpha * err_rear_vel;
end
