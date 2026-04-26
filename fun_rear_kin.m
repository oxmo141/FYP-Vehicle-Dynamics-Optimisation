%% =========================================================================
%  fun_rear_kin — Rear Axle Double-Wishbone Kinematics (Heave Only, FULLY CORRECTED)
%  =========================================================================
%  Computes wheel position and orientation from double-wishbone geometry.
%  Wheel plane defined by ball joints C and F relative to wheel center W.
%  Toe/Camber extracted from actual wheel plane normal.
%
%  INPUTS:
%    phi   : [rad]  lower arm rotation angle (+ = jounce/compression, - = droop/rebound)
%    p     : struct with hardpoints: rvwk (wheel center), rvak/bk/ck (lower), rvdk/ek/fk (upper)
%
%  OUTPUTS:
%    avw   : 3×3 wheel-to-chassis rotation matrix
%    rvwv  : [3×1] wheel center position in chassis frame
%    toe   : [rad] toe angle (ISO 8855: + = toe-in for analyzed wheel)
%    camber: [rad] camber angle (ISO 8855: + = top outboard/negative lean)
%% =========================================================================

function [avw, rvwv, toe, camber] = fun_rear_kin(phi, p)
    
    % Road normal (vertical reference)
    en0 = [0; 0; 1];
    
    % =====================================================================
    %  LOWER ARM ROTATION (about chassis pivots A-B)
    % =====================================================================
    % A: rear chassis pivot (fixed), B: front chassis pivot (fixed)
    % C: ball joint on wheel side (rotates with arm)
    rAB = p.rvbk - p.rvak;                          % Pivot axis vector (fixed)
    rAC_design = p.rvck - p.rvak;                   % Design vector A to C
    axis_lower = rAB / norm(rAB);                   % Normalized rotation axis
    
    % Rodrigues' rotation formula: rotate C about axis by phi
    K_lower = skew_symmetric(axis_lower);
    R_lower = eye(3) + sin(phi)*K_lower + (1-cos(phi))*K_lower^2;
    rAC_rot = R_lower * rAC_design;
    C_pos = p.rvak + rAC_rot;                       % Rotated position of lower ball joint C
    
    % =====================================================================
    %  UPPER ARM ROTATION (about chassis pivots D-E)
    % =====================================================================
    % Similar for upper arm
    rDE = p.rvek - p.rvdk;
    rDF_design = p.rvfk - p.rvdk;
    axis_upper = rDE / norm(rDE);
    
    K_upper = skew_symmetric(axis_upper);
    R_upper = eye(3) + sin(phi)*K_upper + (1-cos(phi))*K_upper^2;
    rDF_rot = R_upper * rDF_design;
    F_pos = p.rvdk + rDF_rot;                       % Rotated position of upper ball joint F
    
    % =====================================================================
    %  WHEEL CENTER POSITION (W): Sphere-Sphere Intersection
    %  W lies on sphere around C with radius |W-C|_design
    %  AND sphere around F with radius |W-F|_design
    % =====================================================================
    W_design = p.rvwk;
    
    % Design radii (fixed lengths from design position)
    len_WC = norm(W_design - p.rvck);
    len_WF = norm(W_design - p.rvfk);
    
    % Intersect spheres: center C_pos radius len_WC, center F_pos radius len_WF
    rvwv = sphere_intersect(C_pos, len_WC, F_pos, len_WF, W_design);
    
    if any(isnan(rvwv))
        warning('Intersection failed at phi = %.3f deg (using design position fallback)', phi*180/pi);
        rvwv = W_design;
    end
    
    % =====================================================================
    %  WHEEL PLANE NORMAL (from ball joints C and F)
    %  Wheel plane passes through W, C, F → normal n = (C - W) × (F - W)
    % =====================================================================
    vec_WC = C_pos - rvwv;  % Vector from wheel center to lower ball joint
    vec_WF = F_pos - rvwv;  % Vector from wheel center to upper ball joint
    
    % Normal to wheel plane (outward-pointing for right wheel)
    n = cross(vec_WC, vec_WF);
    n_norm = norm(n);
    if n_norm < 1e-8
        warning('Degenerate wheel plane at phi = %.3f deg', phi*180/pi);
        n = [0; -1; 0];  % Fallback: lateral outward for right wheel (negative Y)
    else
        n = n / n_norm;
    end
    
    % =====================================================================
    %  CAMBER EXTRACTION (ISO 8855)
    %  Camber = angle between wheel plane normal and lateral (Y) axis,
    %  projected in longitudinal-vertical (XZ) plane.
    %  Positive: top of wheel tilts outboard (n_x > 0 for right wheel)
    % =====================================================================
    % Project normal onto XZ plane (ignore lateral Y for camber)
    n_xz = [n(1); 0; n(3)];
    n_xz = n_xz / norm(n_xz);
    
    % Camber angle from vertical (Z): atan2(longitudinal, vertical)
    % For right wheel, positive camber = positive X tilt
    camber = atan2(n_xz(1), n_xz(3));  % [rad], + = top outboard
    
    % =====================================================================
    %  TOE EXTRACTION (ISO 8855)
    %  Toe = angle of wheel plane's heading from vehicle longitudinal (X),
    %  measured about vertical (Z).
    %  Positive toe-in: wheel front turns toward centerline.
    % =====================================================================
    % Project normal onto horizontal (XY) plane
    n_xy = [n(1); n(2); 0];
    n_xy_norm = norm(n_xy);
    if n_xy_norm < 1e-8
        toe = 0;  % Vertical normal → zero toe
    else
        n_xy = n_xy / n_xy_norm;
        
        % Toe angle: atan2(X component, Y component) of horizontal normal
        % For right wheel, toe-in = positive rotation
        toe = atan2(n_xy(1), n_xy(2));  % [rad], + = toe-in
    end
    
    % =====================================================================
    %  WHEEL ORIENTATION MATRIX (avw: chassis to wheel frame)
    %  Basis: ex (forward/roll), ey (lateral/spin axis = kingpin), ez (radial)
    % =====================================================================
    % Kingpin axis (spin axis): direction from C to F
    kingpin_vec = F_pos - C_pos;
    ey = kingpin_vec / norm(kingpin_vec);  % Wheel spin axis
    
    % Radial direction: along wheel normal n (outward)
    ez = n;  % Already normalized
    
    % Forward direction: right-hand rule, cross spin × radial
    ex = cross(ey, ez);
    ex_norm = norm(ex);
    if ex_norm < 1e-8
        ex = [1; 0; 0];  % Fallback
    else
        ex = ex / ex_norm;
    end
    
    % Ensure right-handed frame
    ez_check = cross(ex, ey);
    if dot(ez_check, ez) < 0
        ez = -ez;
    end
    
    avw = [ex, ey, ez];  % Columns: chassis-to-wheel basis vectors
    
end

%% -----------------------------------------------------------------------
%  SKEW-SYMMETRIC MATRIX FOR RODRIGUES' FORMULA
%% -----------------------------------------------------------------------
function K = skew_symmetric(v)
    K = [ 0,   -v(3),  v(2);
          v(3),  0,   -v(1);
         -v(2),  v(1),  0  ];
end

%% -----------------------------------------------------------------------
%  SPHERE-SPHERE INTERSECTION (analytical solution)
%  Returns point closest to design position p_design
%% -----------------------------------------------------------------------
function p = sphere_intersect(c1, r1, c2, r2, p_design)
    d = norm(c2 - c1);
    
    % Validity checks
    if d > (r1 + r2) + 1e-6 || d + min(r1, r2) < max(r1, r2) - 1e-6
        p = NaN(3,1);
        return;
    end
    if d < 1e-8
        p = NaN(3,1);
        return;
    end
    
    % Distance along line to radical plane
    a = (r1^2 - r2^2 + d^2) / (2 * d);
    
    % Half distance between intersection points
    h = sqrt(r1^2 - a^2);
    if isnan(h) || isreal(h) == 0
        h = 0;
    end
    
    % Center point on line
    dir = (c2 - c1) / d;
    p_center = c1 + a * dir;
    
    % Perpendicular vector (arbitrary in plane, use Z-up preference)
    if abs(dir(3)) < 0.707  % Not too vertical
        perp = cross(dir, [0; 0; 1]);
    else
        perp = cross(dir, [1; 0; 0]);
    end
    perp = perp / norm(perp);
    
    % Two possible points (circle intersection)
    p1 = p_center + (h * perp);
    p2 = p_center - (h * perp);
    
    % Select closest to design
    dist1 = norm(p1 - p_design);
    dist2 = norm(p2 - p_design);
    if dist1 <= dist2
        p = p1;
    else
        p = p2;
    end
end