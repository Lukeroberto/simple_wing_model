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
    Tc = angle_constraint_top(z,p) + angle_constraint_bottom(z,p);
    u = control_law(t,z,u,p);
    A = A_batwing(z,p);
    b = b_batwing(z,u,[Fc Tc],p);

    x = A\b;
    dz(1:3,1) = z(4:6); % dz = [dth1;dth2;dy;ddth1;ddth2;ddy]
    dz(4:6,1) = x;
end

function u = control_law(t,z,u,p)

u = u + 60*sin(pi*t);
   
end

function Fc = contact_force(z,p)

    %% Fixed parameters for contact
    K_c = 10000;
    D_c = 0;
    yC  = 3;
    
    % Position of body
    y  = z(3);
    dy = z(6);
    
    C = y - yC;
    dC = dy;

    if C < 0
        Fc = -K_c*C - D_c*dC;
    else
        Fc = 0;
    end

end

function Tc = angle_constraint_bottom(z,p)

    %% Fixed parameters for contact
    K_c = 10000;
    D_c = 10;
    thC  = -pi/3; % Angle Limit
    
    % Position of body
    th1  = z(1);
    dth1 = z(4);
    
    C = th1 - thC;
    dC = dth1;
    
    if C < 0
        Tc = -K_c*C - D_c*dC;
    else
        Tc = 0;
    end
end

function Tc = angle_constraint_top(z,p)

    %% Fixed parameters for contact
    K_c = 10000;
    D_c = 10;
    thC  = pi/3; % Angle Limit
    
    % Position of body
    th1  = z(1);
    dth1 = z(4);
    
    C = th1 - thC;
    dC = dth1;
    
    if C > 0
        Tc = -K_c*C - D_c*dC;
    else
        Tc = 0;
    end
end