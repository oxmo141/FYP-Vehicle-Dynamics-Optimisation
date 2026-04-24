function psi = uty_trigon(a, b, c)
    % Solve: a*cos(psi) + b*sin(psi) = c
    % Handles complex inputs and near-singular cases
    
    % Force real values (eliminates numerical noise)
    a = real(a);
    b = real(b);
    c = real(c);
    
    % Compute amplitude
    R = sqrt(a^2 + b^2);
    
    % Handle degenerate case: a ≈ b ≈ 0
    if R < 1e-12
        if abs(c) < 1e-12
            psi = 0;  % Equation 0 = 0 (indeterminate)
        else
            psi = NaN;  % Equation 0 = c ≠ 0 (no solution)
        end
        return;
    end
    
    % Normalize c to valid range for arcsin
    ratio = c / R;
    ratio = max(-1.0, min(1.0, ratio));  % Clamp to [-1, 1]
    
    % Compute solution
    if b < 0
        psi = asin(-ratio) - atan2(-a, -b);
    else
        psi = asin(ratio) - atan2(a, b);
    end
    
    % Wrap angle to [-pi, pi]
    psi = mod(psi + pi, 2*pi) - pi;
end