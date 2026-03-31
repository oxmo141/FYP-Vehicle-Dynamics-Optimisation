%% Chassis Contour 
% Plot of Lateral Load Transfer Distribution/ Stiffness Distribution/ Weight Distribution
clear;clc
paramR26;

% parameters car
torsional_rigidity = car.TR * 180/pi; % [Nm/rad]
lat_g = 1;
g= 9.81;
car_mass = car.m;
cg_height = car.cgh;
track = car.track;

% parameters suspension
wheel_radius = car.r_wheel;
front.unsprung_mass = frontunsprung.m;
rear.unsprung_mass = rearunsprung.m;
front.roll_center = front.RC;
rear.roll_center = rear.RC;
sprung_mass = car.m_sprung;

% moments about each axle
front_moment_unsprung = lat_g*g*front.unsprung_mass*(wheel_radius-front.roll_center);
rear_moment_unsprung = lat_g*g*rear.unsprung_mass*(wheel_radius-rear.roll_center);


front_ARB = linspace(front.ARB(1), front.ARB(2), 99);
rear_ARB  = linspace(rear.ARB(1), rear.ARB(2), 99);

front_ARB = [0, front_ARB];
rear_ARB  = [0, rear_ARB];

front_spring = 0.5*car.track^2 * (front.springs * 175.12684);
rear_spring  = 0.5*car.track^2 * (rear.springs * 175.12684);

% Vectorised stiffness matrices
K_front = front_spring' + front_ARB;   % 4x100
K_rear  = rear_spring' + rear_ARB;    % 4x100
%%
% all distributions are front bias
weight_distribution = linspace(0,1,100);
LLTD_val = zeros(length(front_ARB), length(weight_distribution));
stiffness_distribution = zeros(1, length(front_ARB));

count = 1;
for q = 1:size(K_front,1)
    for k = 1:size(K_rear,1)
        for j = 1:size(K_front,2)

            kf = K_front(q,j);
            kr = K_rear(k,j);

            stiffness_distribution(j) = kf/(kf + kr); % ✅ correct

            for i = 1:length(weight_distribution)

                roll_height = cg_height - (1-weight_distribution(i))* ...
                              front.roll_center + weight_distribution(i)*rear.roll_center;

                moment_sprung = lat_g*g*sprung_mass*roll_height;

                A = [kf, 0, -torsional_rigidity;
                     0, kr, torsional_rigidity;
                     1, -1, 1];

                M = [front_moment_unsprung + weight_distribution(i)*moment_sprung;
                     rear_moment_unsprung + (1-weight_distribution(i))*moment_sprung;
                     0];

                R = A\M;

                front_lat_load_transfer = 1/track * ...
                    (torsional_rigidity*R(3) + ...
                    weight_distribution(i)*moment_sprung + front_moment_unsprung);

                rear_lat_load_transfer = 1/track * ...
                    (-torsional_rigidity*R(3) + ...
                    (1-weight_distribution(i))*moment_sprung + rear_moment_unsprung);

                LLTD_val(j,i) = front_lat_load_transfer / ...
                    (front_lat_load_transfer + rear_lat_load_transfer);

            end
        end

        [weight_distribution_mesh, stiffness_distribution_mesh] = ...
            meshgrid(weight_distribution*100, stiffness_distribution*100);
        ContourData(count).weight_distribution = weight_distribution;
        ContourData(count).stiffness_distribution = stiffness_distribution;

        ContourData(count).LLTD = LLTD_val;

        ContourData(count).X = weight_distribution_mesh;
        ContourData(count).Y = stiffness_distribution_mesh;

        ContourData(count).front_spring = front.springs(q);
        ContourData(count).rear_spring  = rear.springs(k);

        % Optional extras
        ContourData(count).K_front = K_front(q,:);
        ContourData(count).K_rear  = K_rear(k,:);

        figure(count)
        contour(weight_distribution_mesh, stiffness_distribution_mesh, ...
                LLTD_val*100, 10, 'LineWidth',1.5);

        colormap(jet)
        colorbar

        title(sprintf('Front Spring = %.2f lb/in, Rear Spring = %.2f lb/in', ...
              front.springs(q), rear.springs(k)));

        xlabel('Static weight dist front [%]')
        ylabel('Stiffness dist front [%]')

        grid on
        count = count + 1;
    end
end