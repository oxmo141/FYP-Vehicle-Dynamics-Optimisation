function [R, M] = roll_angle(lat_g, car, front, rear, frontunsprung, rearunsprung)
    % roll_angle: Calculates the roll angles for the front and rear axles of a vehicle
    % based on lateral acceleration and vehicle parameters.
    %
    % Inputs:
    %   lat_g: Lateral acceleration in g's (e.g., 1 for 1g).
    %   car: Structure containing car parameters:
    %       - .r_wheel: Wheel radius (m).
    %       - .m_sprung: Sprung mass of the vehicle (kg).
    %       - .h_roll: Height of the center of mass (m).
    %       - .a: Distance from the front axle to the center of mass (m).
    %       - .b: Distance from the rear axle to the center of mass (m).
    %       - .TR: Track width (m).
    %   front: Structure containing front suspension parameters:
    %       - .k_roll: Front roll stiffness (Nm/rad).
    %       - .RC: Roll center height (m).
    %   rear: Structure containing rear suspension parameters:
    %       - .k_roll: Rear roll stiffness (Nm/rad).
    %       - .RC: Roll center height (m).
    %   frontunsprung: Structure containing front unsprung mass parameters:
    %       - .m: Unsprung mass of the front axle (kg).
    %   rearunsprung: Structure containing rear unsprung mass parameters:
    %       - .m: Unsprung mass of the rear axle (kg).
    %
    % Outputs:
    %   R: Roll angles for front and rear axles (radians).
    %   M: Load matrix used in roll angle calculations (N*m).
 
    g = 9.81; % Acceleration due to gravity (m/s^2)
 
    % Calculate moments about each axle
    Moment.frontunsprung = lat_g * g * frontunsprung.m * (car.r_wheel - front.RC);
    Moment.rearunsprung  = lat_g * g * rearunsprung.m * (car.r_wheel - rear.RC);
    Moment.sprung        = lat_g * g * car.m_sprung * car.h_roll;
 
    % Stiffness matrix
    A = [front.k_roll, 0, -car.TR;  % Front axle contribution
         0, rear.k_roll,  car.TR;  % Rear axle contribution
         1, -1, 1];                % Constraint equation
 
    % Load matrix
    M = [Moment.frontunsprung + car.b/(car.a + car.b) * Moment.sprung;  % Front load
         Moment.rearunsprung  + car.a/(car.a + car.b) * Moment.sprung;  % Rear load
         0];                                                            % Constraint for equilibrium
 
    % Solve for roll angles
    R = A \ M; % Using matrix left division to solve Ax = b
 
end