function derive_model()

name = 'batwing';

%% Define symbolic variables and joint positions
syms t y dy ddy th1 dth1 ddth1 th2 th2_0 dth2 ddth2 tau F_d F_y l1 l2 k m1 m2 mb I1 I2 rho c_d g real

% Grouping 
q   = [th1; th2; y];
dq  = [dth1; dth2; dy];
ddq = [ddth1; ddth2; ddy];

u = tau;
Fc = F_y;
p = [l1;l2;m1;m2;mb;I1;I2;k;th2_0;rho;c_d;g];

%% Key Vectors and Derivatives
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat, jhat);

% Other unit vectors
e1hat = cos(th1)*ihat + sin(th1)*jhat;
e2hat = cos(th1 + th2)*ihat + sin(th1 + th2)*jhat;

% Vector derivative function handle
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq];

% Location of keypoints
r_OA = y*jhat; 
r_AB = r_OA + l1*e1hat;
r_AB_cm = r_OA + 0.5*l1*e1hat;
r_BC = r_AB + l2*e2hat;
r_BC_cm = r_AB + 0.5*l2*e2hat;

keypoints = [r_OA r_AB r_BC];

% Derivatives of important locations
dr_OA    = ddt(r_OA);
dr_AB    = ddt(r_AB);
dr_AB_cm = ddt(r_AB_cm);
dr_BC    = ddt(r_BC);
dr_BC_cm = ddt(r_BC_cm);

%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces

% F2Q calculates the contribution of a force to all generalized forces
% for forces, F is the force vector and r is the position vector of the 
% point of force application
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));

% M2Q calculates the contribution of a moment to all generalized forces
% M is the moment vector and w is the angular velocity vector of the
% body on which the moment acts
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));

% Kinetic Energies
Tb = (1/2)*mb*dot(dr_OA,dr_OA);
T1 = (1/2)*m1*dot(dr_AB_cm,dr_AB_cm) + (1/2)*I1*dth1^2;
T2 = (1/2)*m2*dot(dr_BC_cm,dr_BC_cm) + (1/2)*I2*dth2^2;


% Potential Energies
Vb = mb*g*dot(r_OA,jhat);
V1 = m1*g*dot(r_AB,jhat);
V2 = m2*g*dot(r_BC,jhat);
Vk = (1/2)*k*(th2-th2_0)^2;

% Define drag force 
unit_vec = -dr_AB/norm(dr_AB);
F_d = (1/2)*c_d*rho*norm(dr_AB)^2*unit_vec;

% Generalized forces
QFd = F2Q(F_d, r_AB);
QFc = F2Q(Fc*jhat , r_OA);
QM = M2Q(-tau*khat, -th1*khat);

% Sum contributions
T = Tb + T1 + T2;
V = Vb + V1 + V2 + Vk;
Q = QFd + QFc + QM;

% Calculate rcm, the location of the center of mass
rcm = (mb*r_OA + m1*r_AB_cm + m2*r_BC_cm)/(mb+m1+m2);

% Assemble C, the set of constraints
C = y;  % When y = 0, the constraint is satisfied because wing touching ground
dC= ddt(C);

%% All the work is done!  Just turn the crank...
%%% Derive Energy Function and Equations of Motion
E = T+V;                                         % total system energy
L = T-V;                                         % the Lagrangian
eom = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;  % form the dynamics equations

size(eom)

%%% Rearrange Equations of Motion. 
A = jacobian(eom,ddq);
b = A*ddq - eom;


%%% Write functions to evaluate dynamics, etc...
z = sym(zeros(length([q;dq]),1)); % initialize the state vector
z(1:3,1) = q;
z(4:6,1) = dq;

% Write functions to a separate folder because we don't usually have to see them
directory = 'AutoDerived/';
% Write a function to evaluate the energy of the system given the current state and parameters
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% Write a function to evaluate the A matrix of the system given the current state and parameters
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
% Write a function to evaluate the b vector of the system given the current state, current control, and parameters
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u Fc p});

matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});

matlabFunction(C,'file',[directory 'C_' name],'vars',{z u p});
matlabFunction(dC,'file',[directory 'dC_' name],'vars',{z u p});

% Write a function to evaluate location of main body
% body = [r_OA(1:2); dr_OA(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
% matlabFunction(body,'file',[directory 'body_' name],'vars',{z p});

