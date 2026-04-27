function [avw, rvwv, toe, camber] = fun_rear_kin(phi, p, toe_prev)
% =========================================================================
%  REAR SUSPENSION KINEMATICS
%  Inputs:
%    phi      - lower arm rotation angle (rad)
%    p        - hardpoint parameter struct
%    toe_prev - toe angle from previous phi step (rad), for branch continuity
%               Pass [] or omit on first call
% =========================================================================

% --- Branch continuity seed -------------------------------------------
if nargin < 3 || isempty(toe_prev)
    toe_prev = 0;
end

% =====================================================================
%  LOWER ARM — driven by phi (input angle)
% =====================================================================
rAB        = p.rvbk - p.rvak;
rAC_design = p.rvck - p.rvak;
axis_lower = rAB / norm(rAB);
K_lower    = skew_symmetric(axis_lower);
R_lower    = eye(3) + sin(phi)*K_lower + (1-cos(phi))*K_lower^2;
C_pos      = p.rvak + R_lower * rAC_design;

% =====================================================================
%  UPPER ARM — Newton-Raphson to close kinematic chain
% =====================================================================
rDE        = p.rvek - p.rvdk;
rDF_design = p.rvfk - p.rvdk;
axis_upper = rDE / norm(rDE);
K_upper    = skew_symmetric(axis_upper);
len_CF     = norm(p.rvck - p.rvfk);

phi_upper  = phi;  % initial guess

for iter_chain = 1:60
    R_upper = eye(3) + sin(phi_upper)*K_upper + (1-cos(phi_upper))*K_upper^2;
    F_pos   = p.rvdk + R_upper * rDF_design;
    dist_CF = norm(C_pos - F_pos);
    res_CF  = dist_CF - len_CF;

    if abs(res_CF) < 1e-10, break, end

    dF_dphi = (cos(phi_upper)*K_upper + sin(phi_upper)*K_upper^2) * rDF_design;
    CF_vec  = F_pos - C_pos;
    J       = dot(CF_vec, dF_dphi) / dist_CF;
    if abs(J) < 1e-14, break, end

    step      = max(-0.05, min(0.05, res_CF / J));
    phi_upper = phi_upper - step;
end

R_upper = eye(3) + sin(phi_upper)*K_upper + (1-cos(phi_upper))*K_upper^2;
F_pos   = p.rvdk + R_upper * rDF_design;

% =====================================================================
%  WHEEL CENTER — sphere-sphere intersection
% =====================================================================
len_WC = norm(p.rvwk - p.rvck);
len_WF = norm(p.rvwk - p.rvfk);

rvwv = sphere_intersect(C_pos, len_WC, F_pos, len_WF, p.rvwk);
if any(isnan(rvwv))
    rvwv = p.rvwk;
    warning('fun_rear_kin: sphere_intersect failed at phi=%.4f', phi)
end

% =====================================================================
%  DESIGN UPRIGHT FRAME  (computed once from design hardpoints)
%  This is the reference frame for:
%    (a) branch-flip prevention
%    (b) H offset reconstruction
% =====================================================================
kingpin_des = p.rvfk - p.rvck;
ey0_des     = kingpin_des / norm(kingpin_des);

vec_CW_des  = p.rvwk - p.rvck;
n_lat_des   = vec_CW_des - dot(vec_CW_des, ey0_des) * ey0_des;
if norm(n_lat_des) > 1e-8
    ez0_des = n_lat_des / norm(n_lat_des);
else
    ez0_des = [0; -1; 0];
end

ex0_des = cross(ey0_des, ez0_des);
if norm(ex0_des) > 1e-8
    ex0_des = ex0_des / norm(ex0_des);
end
% Reorthogonalize design frame
ez0_des = cross(ex0_des, ey0_des);
ez0_des = ez0_des / norm(ez0_des);

% =====================================================================
%  CURRENT UPRIGHT FRAME  (from current C, F, W positions)
% =====================================================================
kingpin_vec = F_pos - C_pos;
ey0         = kingpin_vec / norm(kingpin_vec);

vec_CW = rvwv - C_pos;
n_lat  = vec_CW - dot(vec_CW, ey0) * ey0;
if norm(n_lat) > 1e-8
    ez0 = n_lat / norm(n_lat);
else
    ez0 = [0; -1; 0];
end

ex0 = cross(ey0, ez0);
if norm(ex0) > 1e-8
    ex0 = ex0 / norm(ex0);
else
    ex0 = [1; 0; 0];
end
% Reorthogonalize
ez0 = cross(ex0, ey0);
ez0 = ez0 / norm(ez0);

% =====================================================================
%  BRANCH CONSISTENCY — align current frame to design frame
%  Uses dot product test (stable across full travel range)
% =====================================================================
if dot(ex0, ex0_des) < 0
    ex0 = -ex0;
    ez0 = -ez0;
end
if dot(ey0, ey0_des) < 0
    ey0 = -ey0;
    ex0 = cross(ey0, ez0);
    ex0 = ex0 / norm(ex0);
    ez0 = cross(ex0, ey0);
    ez0 = ez0 / norm(ez0);
end

% =====================================================================
%  TOE LINK — H tracks upright rigidly, solve for toe rotation
% =====================================================================
if isfield(p, 'rvgk') && isfield(p, 'rvhk')

    G_pos  = p.rvgk;
    H_des  = p.rvhk;
    len_GH = norm(H_des - G_pos);

    % Express H offset in design upright LOCAL frame
    R_des          = [ex0_des, ey0_des, ez0_des];   % 3x3, columns = axes
    H_offset_local = R_des' * (H_des - p.rvwk);     % R^T = world->local

    % Reconstruct H in current upright frame (before toe rotation)
    R_cur     = [ex0, ey0, ez0];
    H_upright = rvwv + R_cur * H_offset_local;

    % --- Diagnostic at design position --------------------------------
    if phi == 0
        fprintf('\n--- H TRACKING DIAGNOSTIC (phi=0) ---\n')
        fprintf('H design:           [%.5f, %.5f, %.5f]\n', H_des(1),      H_des(2),      H_des(3))
        fprintf('H_offset_local:     [%.5f, %.5f, %.5f]\n', H_offset_local(1), H_offset_local(2), H_offset_local(3))
        fprintf('H_upright (no toe): [%.5f, %.5f, %.5f]\n', H_upright(1),  H_upright(2),  H_upright(3))
        fprintf('H_design==H_upright? dist=%.2e mm\n',      norm(H_upright - H_des)*1000)
        fprintf('||H_upright - G|| = %.6f m  (target = %.6f m)\n', norm(H_upright - G_pos), len_GH)
        fprintf('--------------------------------------\n\n')
    end

    % ---- Solve toe rotation that satisfies ||H_rotated - G|| = len_GH
    %      PASS toe_prev for branch continuity  <-- KEY FIX
    toe_rad = solve_toe_angle(rvwv, ey0, H_upright, G_pos, len_GH, toe_prev);

    % Apply toe rotation to upright frame vectors
    K_ey     = skew_symmetric(ey0);
    R_toe    = eye(3) + sin(toe_rad)*K_ey + (1-cos(toe_rad))*K_ey^2;

    ex_final = R_toe * ex0;
    ey_final = ey0;          % Kingpin axis is unchanged by toe
    ez_final = R_toe * ez0;

else
    toe_rad  = 0;
    ex_final = ex0;
    ey_final = ey0;
    ez_final = ez0;
end

% =====================================================================
%  OUTPUT FRAME
% =====================================================================
avw = [ex_final, ey_final, ez_final];

% =====================================================================
%  TOE ANGLE  (projected to ground plane XY)
%  atan2 convention:
%    Right wheel: ex_final(2) < 0 → toe-in  (negative Y = inboard)
%    Left  wheel: mirrored hardpoints produce opposite sign automatically
% =====================================================================
ex_xy = ex_final(1:2);
ex_xy = ex_xy / norm(ex_xy);
toe   = atan2(ex_xy(2), ex_xy(1));

% Wrap to (-pi/2, +pi/2]
if toe >  pi/2, toe = toe - pi; end
if toe < -pi/2, toe = toe + pi; end

% =====================================================================
%  CAMBER ANGLE  (tilt about longitudinal axis)
% =====================================================================
camber = atan2(ey_final(1), ey_final(3));

% =====================================================================
%  DIAGNOSTIC PRINT
% =====================================================================
fprintf('phi=%7.4f rad | ex_final=[%+.6f, %+.6f, %+.6f] | toe=%+.4f rad (%+.2f deg) | camber=%+.4f rad (%+.2f deg)\n', ...
    phi, ex_final(1), ex_final(2), ex_final(3), ...
    toe,   toe*180/pi, ...
    camber, camber*180/pi)

end  % fun_rear_kin


% =========================================================================
%  SOLVE TOE ANGLE
%  Rotates H about the kingpin (ey) until ||H_rotated - G|| = len_GH.
%  Two branches are tried; the one CLOSEST to toe_prev is selected.
%  This enforces smooth, continuous, branch-consistent behaviour across
%  the full phi range and between left/right wheels.
% =========================================================================
function toe = solve_toe_angle(W_pos, ey, H_upright, G_pos, len_GH, toe_prev)

if nargin < 6 || isempty(toe_prev)
    toe_prev = 0;
end

K_ey     = skew_symmetric(ey);
H_offset = H_upright - W_pos;
tol      = 1e-11;
max_iter = 100;

% --- Inner Newton solver (reusable) -----------------------------------
    function t = newton_toe(t0)
        t = t0;
        for it = 1:max_iter
            R_t   = eye(3) + sin(t)*K_ey + (1-cos(t))*K_ey^2;
            H_t   = W_pos + R_t * H_offset;
            d     = norm(H_t - G_pos);
            res   = d - len_GH;
            if abs(res) < tol, return, end
            dHdt  = (cos(t)*K_ey + sin(t)*K_ey^2) * H_offset;
            J     = dot(H_t - G_pos, dHdt) / (d + 1e-14);
            if abs(J) < 1e-14, return, end
            step  = max(-0.1, min(0.1, res / J));
            t     = t - step;
        end
    end
% ----------------------------------------------------------------------

% Branch 1: seed from toe_prev  (most likely correct branch)
toe1 = newton_toe(toe_prev);
R1   = eye(3) + sin(toe1)*K_ey + (1-cos(toe1))*K_ey^2;
res1 = abs(norm(W_pos + R1*H_offset - G_pos) - len_GH);

% Branch 2: seed from opposite side
toe2 = newton_toe(-toe_prev);
R2   = eye(3) + sin(toe2)*K_ey + (1-cos(toe2))*K_ey^2;
res2 = abs(norm(W_pos + R2*H_offset - G_pos) - len_GH);

% --- Select branch closest to toe_prev (continuity criterion) ---------
%     Only fall back to residual comparison if one branch fails to converge
converged1 = res1 < 1e-8;
converged2 = res2 < 1e-8;

if converged1 && converged2
    % Both converged: pick the one closest to previous value
    if abs(toe1 - toe_prev) <= abs(toe2 - toe_prev)
        toe = toe1;
    else
        toe = toe2;
    end
elseif converged1
    toe = toe1;
elseif converged2
    toe = toe2;
else
    % Neither converged cleanly: best residual wins
    if res1 <= res2
        toe = toe1;
    else
        toe = toe2;
    end
end

end  % solve_toe_angle


% =========================================================================
%  SKEW-SYMMETRIC MATRIX
% =========================================================================
function K = skew_symmetric(v)
K = [  0,    -v(3),  v(2);
       v(3),  0,    -v(1);
      -v(2),  v(1),  0   ];
end


% =========================================================================
%  SPHERE-SPHERE INTERSECTION
%  Returns the point on the intersection circle closest to p_design.
% =========================================================================
function p = sphere_intersect(c1, r1, c2, r2, p_design)

d = norm(c2 - c1);

if d < 1e-8 || d > (r1+r2)+1e-6 || (d+min(r1,r2)) < (max(r1,r2)-1e-6)
    p = NaN(3,1);
    return
end

a   = (r1^2 - r2^2 + d^2) / (2*d);
h   = sqrt(max(0, r1^2 - a^2));
dir = (c2 - c1) / d;
pc  = c1 + a*dir;

if abs(dir(3)) < 0.707
    perp1 = cross(dir, [0;0;1]);
else
    perp1 = cross(dir, [1;0;0]);
end
perp1 = perp1 / norm(perp1);
perp2 = cross(dir, perp1);

v_des = p_design - pc;
theta = atan2(dot(v_des, perp2), dot(v_des, perp1));

p = pc + h*(cos(theta)*perp1 + sin(theta)*perp2);

end
