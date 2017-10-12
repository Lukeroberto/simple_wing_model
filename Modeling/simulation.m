function [tout, zout] = simulation(z0,u,p,tspan)

t0 = tspan(1); tend = tspan(end);   % set initial and final times

%% Setup tolerance Options
inttol = 1e-6;  % set integration tolerances
% iphase = 1;     % phase number is currently phase 1
% sols = [];      % initialize array of solution structures
opts = odeset('abstol',inttol,'reltol',inttol);

f = @(t,z) continuous_dynamics(t,z,u,p);
tic
sol = ode45(f,tspan,z0,opts);
toc

tout = sol.x;
zout = sol.y;

end

%% Continuous Dynamics
function dz = continuous_dynamics(t,z,u,p)
 
    % Contact model
    Fc = contact_force(z,p);
    A = A_batwing(z,p);
    b = b_batwing(z,u,Fc,p);

    x = A\b;
    dz(1:3,1) = z(4:6); % dz = [dth1;dth2;dy;ddth1;ddth2;ddy]
    dz(4:6,1) = x;
end

function Fc = contact_force(z,p)

    %% Fixed parameters for contact
    K_c = 10000;
    D_c = 0;
    yC  = 0;
    
    % Position of body
    y  = z(3);
    dy = z(6);

    % a. Compute constraint C which gives height of foot relative to ground
    C = y - yC;

    % b. Compute constraint rate, \dot{C}
    dC = dy;

    % c. Set Fc based on compliant contact model
    % d. If foot is above the ground, or Fc<0, set Fc = 0

    if C < 0
        Fc = -K_c*C - D_c*dC;
    else
        Fc = 0;
    end

end
