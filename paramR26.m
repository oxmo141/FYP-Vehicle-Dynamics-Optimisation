%% All Parameters of the FSAE Car
g = 9.81;

%% Distributions
car.front_m = 67+66;
car.rear_m = 66.5+64.5;
car.m = car.front_m + car.rear_m;
weight_distribution = car.front_m/car.m; %front bias

ROLL_INERTIA = 182.24965;
YAW_INERTIA = 226.2991478;

%% Unsprung mass of 1 corner
frontunsprung.m = 9.3778;
rearunsprung.m = 9.247;

car.m_unsprung = (frontunsprung.m + rearunsprung.m) * 2;
%% Car/ Sprung (Appendix A: diagram of car parameters)
car.wheelbase = 1.558;
car.track = 1.21;
car.cgh = 0.303736;
car.r_wheel = 0.2032;
car.m_sprung = car.m - 2*frontunsprung.m - 2*rearunsprung.m;
car.rollover = (car.track * 0.5) / car.cgh;

car.b = (car.wheelbase * car.m * weight_distribution) / car.m;
car.a = car.wheelbase - car.b;

%% Sprung mass of 1 corner
frontsprung.m = (0.5 * (weight_distribution) * car.m) - frontunsprung.m;
rearsprung.m = (0.5 * ((1-weight_distribution)) * car.m) - rearunsprung.m;

frontunsprung.percentage = (frontunsprung.m / (frontunsprung.m + frontsprung.m)) * 100;
rearunsprung.percentage = (rearunsprung.m / (rearunsprung.m + rearsprung.m)) * 100;

car.sprungCGH = (car.m / car.m_sprung) * car.cgh - (2 * frontunsprung.m / car.m_sprung) * car.r_wheel ...
            - (2 * rearunsprung.m / car.m_sprung) * car.r_wheel;
car.sprung_b = (car.m * car.b - 2 * frontunsprung.m * car.wheelbase) / car.m_sprung;
car.sprung_a = car.wheelbase - car.sprung_b;

%% Chassis and Suspension Stiffness 
car.TR = (3047.62 + 2912.586)/2 * 180/pi; 

front.springs = [200, 225, 250, 300]; % Collection of springs to use
rear.springs = [200, 225, 250, 300];

front.ARB = [527.36, 644.1] * 180/pi; % Minimum to maximum value 
rear.ARB = [505.68, 771.59] * 180/pi; % of ARB stiffness [Nm/rad of roll]

front.ks = front.springs(2) * 175.12684; % spring rate
rear.ks = rear.springs(1) * 175.12684;

front.cs = 4050.8468 * 2/ (car.track)^2; %780
rear.cs = 3705.0427 * 2/ (car.track)^2; %

car.Kt = (10^3)/0.0122; % tyre radial rate
front.MR = 1.0; % motion ratio of coil over
rear.MR = 1.0;

front.kw = front.ks*front.MR^2; % wheel rate
rear.kw = rear.ks*rear.MR^2;
front.Kr = (front.kw*car.Kt) / (front.kw+car.Kt); %ride rate
rear.Kr = (rear.kw*car.Kt) / (rear.kw+car.Kt);

front.RC = 0.035;
rear.RC = 0.050;
car.h_roll = car.cgh - ((car.a*front.RC + car.b*rear.RC)/(car.a+car.b));

front.ks_roll = 0.5*front.kw*car.track^2; % spring roll rate Nm/rad
rear.ks_roll = 0.5*rear.kw*car.track^2;

front.cs_roll = 0.5*front.cs*car.track^2;
rear.cs_roll = 0.5*rear.cs*car.track^2;

[front.target_ARB, rear.target_ARB] = targetARBstiffness(50, front, rear);
front.k_roll = front.ks_roll + front.target_ARB;
rear.k_roll = rear.ks_roll + rear.target_ARB;

front.ARB_setting = (front.target_ARB - min(front.ARB)) / ...
                    (min(front.ARB) + max(front.ARB)); 
rear.ARB_setting = (rear.target_ARB - min(rear.ARB)) / ...
                    (min(rear.ARB) + max(rear.ARB)); 

%% Natural Frequency and Critical Damping
frontunsprung.hop_freq = (1/(2*pi)) * sqrt(front.Kr / frontunsprung.m);
rearunsprung.hop_freq =  (1/(2*pi)) * sqrt(rear.Kr / rearunsprung.m);

