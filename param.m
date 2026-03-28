%% All Parameters of the FSAE Car
g = 9.81;

%% Distributions
weight_distribution = 50.95; %front bias

%% Unsprung mass of 1 corner
frontunsprung.m = 12.15;
rearunsprung.m = 13.35;

%% Car/ Sprung (Appendix A: diagram of car parameters)
car.wheelbase = 1.558;
car.track = 1.21;
car.cgh = 0.256;
car.m = 270;
car.r_wheel = 0.23707;
car.m_sprung = car.m - 2*frontunsprung.m - 2*rearunsprung.m;

car.b = (car.wheelbase * car.m * weight_distribution/100) / car.m;
car.a = car.wheelbase - car.b;

%% Sprung mass of 1 corner
frontsprung.m = (0.5 * (weight_distribution/100) * car.m) - frontunsprung.m;
rearsprung.m = (0.5 * ((100-weight_distribution)/100) * car.m) - rearunsprung.m;

%% Chassis and Suspension Stiffness 
car.TR = (3047.62 + 2912.586)/2 * 180/pi; 

front.springs = [200, 225, 250, 300];
rear.springs = [200, 225, 250, 300];

front.ks = front.springs(4) * 175.12684; % spring rate
rear.ks = rear.springs(4) * 175.12684;

front.cs = 5000;
rear.cs = 3000;

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

front.ARB = 0;
rear.ARB = 0;
front.k_roll = front.ks_roll+front.ARB;
rear.k_roll = rear.ks_roll+rear.ARB;

%% Natural Frequency and Critical Damping
% frontsprung.omega = (1/(2*pi)) * sqrt(front.Kr/car.m_sprung);
% rearsprung.omega =  (1/(2*pi)) * sqrt(rear.Kr/car.m_sprung);

