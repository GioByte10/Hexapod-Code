clear; close all; clc;
syms LA LD LB LC LC1 LN real        % Link lengths
syms qA qB qC qD real         % Input joint angles
syms xbc ybc xe ye real 
%% Define base joint positions
Ao = [0; 0];
Do = [LN; 0];

%% Position of passive joints connected to actuators
Bo = Ao + LA * [cos(qA); sin(qA)];
Co = Do + LD * [cos(qD); sin(qD)];

%% Distance constraints connecting B & C links
eq1 = (xbc - Bo(1))^2 + (ybc - Bo(2))^2 == LB^2;
eq2 = (xbc - Co(1))^2 + (ybc - Co(2))^2 == LC1^2;

%% Solve the equations to find BC
S = solve([eq1, eq2], [xbc, ybc], Real=true);

%BC position in the -y domain
BC_down=[simplify(S.xbc(2)); simplify(S.ybc(2))];
%BC position in the +y domain
BC_up=[simplify(S.xbc(2)); simplify(S.ybc(2))];

%% Find the coordinates of the center of the foot E
E_down=((BC_down-Co)/LC1)*LC+Co;
E_up=((BC_down-Co)/LC1)*LC+Co;
%% Calculate Jacobian for BC & E
J_BC = [diff(BC_down(1),qA), diff(BC_down(1),qD); diff(BC_down(2),qA), diff(BC_down(2),qD)];
J_E =[diff(E_down(1),qA), diff(E_down(1),qD); diff(E_down(2),qA), diff(E_down(2),qD)];

%% Saving Functions
matlabFunction(E_down,BC_down,Bo,Co,E_up,BC_up,"File","Five_bar_position_rigid_foot");
matlabFunction(J_BC,"File","Five_bar_Jacobian_BC");
matlabFunction(J_E,"File","Five_bar_Jacobian_Rigid_Foot");