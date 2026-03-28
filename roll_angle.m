function [R, M] = roll_angle(lat_g, car, front, rear, frontunsprung, rearunsprung)

g = 9.81;

% moments about each axle
Moment.frontunsprung = lat_g*g*frontunsprung.m*(car.r_wheel-front.RC);
Moment.rearunsprung  = lat_g*g*rearunsprung.m*(car.r_wheel-rear.RC);
Moment.sprung        = lat_g*g*car.m_sprung*car.h_roll;

% stiffness matrix
A = [front.k_roll, 0, -car.TR;
     0, rear.k_roll,  car.TR;
     1, -1, 1];

% load matrix
M = [Moment.frontunsprung + car.b/(car.a+car.b)*Moment.sprung;
     Moment.rearunsprung  + car.a/(car.a+car.b)*Moment.sprung;
     0];

% solve for roll angles
R = A \ M;

end