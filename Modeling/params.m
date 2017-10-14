function p = params()
l1    = 1; % Length of first arm
l2    = 1; % Length of second arm
A     = 1; % Area of wing
m1    = 1; % Mass of first arm
m2    = 1; % Mass of second arm
mb    = 1; % Mass of base
I1    = 1; % Inertia of first arm
I2    = 1; % Inertia of second arm
k     = 10000; % Torsional spring constant
th2_0 = -pi/4; % Initial torsional spring angle
rho   = 1; % Density of air
c_d   = 1; % Drag coefficient
g     = 9.81; % m/s^2
p     = [l1;l2;A;m1;m2;mb;I1;I2;k;th2_0;rho;c_d;g];
end