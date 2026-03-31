function [found_front_ARB, found_rear_ARB] = targetARBstiffness(targetstiffness_percentage, front_struct, rear_struct)
    % targetARBstiffness: Finds front and rear ARB stiffness values that
    %                     achieve a specified target stiffness distribution.
    %
    % Inputs:
    %   targetstiffness_percentage: The desired front-axle roll stiffness
    %                               distribution as a percentage (e.g., 50 for 50%).
    %   front_struct: A struct containing:
    %     .ks_roll: Inherent front roll stiffness (Nm/rad).
    %     .ARB_range: [min_ARB_stiffness, max_ARB_stiffness] for the front ARB (Nm/rad).
    %   rear_struct: A struct containing:
    %     .ks_roll: Inherent rear roll stiffness (Nm/rad).
    %     .ARB_range: [min_ARB_stiffness, max_ARB_stiffness] for the rear ARB (Nm/rad).
    %
    % Outputs:
    %   found_front_ARB: The front ARB stiffness (Nm/rad) that achieves the target, or NaN if not found.
    %   found_rear_ARB: The rear ARB stiffness (Nm/rad) that achieves the target, or NaN if not found.

    % Initialize output variables to NaN (Not a Number)
    % These will remain NaN if no solution is found within the specified ranges.
    found_front_ARB = NaN;
    found_rear_ARB = NaN;

    % Define a search step size for ARB stiffness.
    % Adjust this value to balance precision and computational time.
    step_size = 0.5; % Example: Iterate every 0.5 Nm/rad

    % Convert the target stiffness percentage to a ratio for calculation.
    target_ratio = targetstiffness_percentage / 100;

    % Extract the ARB stiffness ranges from the input structs.
    % Ensure these ranges are in consistent units (e.g., Nm/rad).
    front_ARB_min = front_struct.ARB(1);
    front_ARB_max = front_struct.ARB(2);
    rear_ARB_min = rear_struct.ARB(1);
    rear_ARB_max = rear_struct.ARB(2);

    % Define a small tolerance for floating-point comparisons.
    % Due to the nature of floating-point arithmetic, direct equality checks (==) can be unreliable.
    tolerance = 1e-6; % Values within this difference are considered equal.

    % Loop through possible front ARB stiffness values within the specified range.
    for current_front_ARB = front_ARB_min : step_size : front_ARB_max

        % Loop through possible rear ARB stiffness values within the specified range.
        for current_rear_ARB = rear_ARB_min : step_size : rear_ARB_max

            % Calculate the total roll stiffness for the front and rear axles
            % by adding the inherent stiffness to the current ARB stiffness.
            total_front_stiffness = front_struct.ks_roll + current_front_ARB;
            total_rear_stiffness = rear_struct.ks_roll + current_rear_ARB;

            % Calculate the total system roll stiffness.
            denominator = total_front_stiffness + total_rear_stiffness;

            % Check for potential division by zero. If the total stiffness is effectively zero,
            % this combination is invalid, and we skip to the next iteration.
            if abs(denominator) < eps % 'eps' is a small positive number, useful for floating-point comparisons to zero.
                continue;
            end

            % Calculate the stiffness distribution ratio.
            stiffness_distribution_ratio = total_front_stiffness / denominator;

            % Compare the calculated distribution to the target ratio using a tolerance.
            if abs(stiffness_distribution_ratio - target_ratio) < tolerance
                % If a match is found, assign the current ARB values to the outputs.
                found_front_ARB = current_front_ARB;
                found_rear_ARB = current_rear_ARB;
                return; % Exit the function immediately as a solution has been found.
            end
        end
    end

    % If the function completes without finding a solution,
    % found_front_ARB and found_rear_ARB will remain NaN.
end
