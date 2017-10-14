clear

setpath; % setup the path to all the nested files

% Simulation
p = params();

z0 = [0;0;3;0;0;0];
u = 0;
tf = 10;

[t, z] = simulation(z0,u,p,[0 tf]); % run simulation

% Run the animation
figure(2)            % get the coordinates of the points to animate
speed = .25;         % set animation speed
clf                  % clear fig
animate(t,z,p,speed) % run animation
