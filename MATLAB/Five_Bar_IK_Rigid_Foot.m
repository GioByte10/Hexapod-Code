% This is a file to perform Inverse Kinematics on the leg with a rigid foot
clear; clc; close all;
addpath('Functions')
load('I_motor.mat') %load spped-current curve
%% Set File Name to Save Variables
File_Name='cycle_shortHS.mat';
PlotTitle='Kinematics Plot';
%% Set Lengths
LA=.06; LD=LA;
LC1=.19; LB=LC1;
LC2=.05;
LC=LC1+LC2;
LN=.15;
r_foot=.02;
%% Set Walk Speed
V_Stride=.24;
%% Set Motor Parameters
Gear_Ratio=5;
K_i=.12; % [Nm/A]
%% Set Ao and Do
Ao=[0;0];
Do=[LN;0];
%% Define Walk Cycle Parameters
Step_Height=.06; 
disp('Finding Workspace')
ShrinkFactor=.3; %set between 0-1 0 is convex, 1 is compact
WorkspaceQuality=200; %resolution of angles to check in workspace
CheckBound=0; %Set to 1 if you would like to see all points simulated to verify a good boundry has been found

Workspace=limitsE(LA,LB,LC,LC1,LD,LN,WorkspaceQuality,ShrinkFactor,CheckBound);
disp('Finding Workspace Rectangle')
[maxLength,bestRect] = WorkspaceRectangle(Workspace,Step_Height);
Step_Length=maxLength;                                                                                                                                                                                                         4;
Stride_y_pos=min(bestRect.Vertices(:,2)); %Position of the bottom of the walk cycle
Stride_x_pos=(bestRect.Vertices(1,1)+bestRect.Vertices(3,1))/2; %Position of the center of walk cycle
Smoothing_r=Step_Height/5; %Radius of the arc that smooths between stride and ellipse
np=100; %Number of points about the walk cycle to plot (resolution) 

%% Calculate Walk Cycle Coordinates
disp('Calculating Walk Cycle Coordinates')
[Walk_Cycle_x, Walk_Cycle_y, Point_Distance, Cycle_Length] = WalkCycle(np, Stride_x_pos, Stride_y_pos, Step_Height, Step_Length, Smoothing_r);

stride_point_end=0;
for i=1:np
    if Walk_Cycle_y(i)==Stride_y_pos
        stride_point_end=i;
    else
        break
    end
end

T=Cycle_Length/V_Stride;
dt=Point_Distance/V_Stride;
t=0:dt:(np-1)*dt;
%% Perform Inverse Kinematics
options = optimoptions('fsolve','Display','none');
q0=[pi 0 0 pi/2];
WalkE=zeros(np,2);
% Initializing vector to save angles
qA=zeros(np,1);
qD=zeros(np,1);
Cycle_Poses=zeros(2,6,np); % Initializing Array to Save Poses
F=zeros(2,np);
BC_vec=zeros(2,np);
Jacobian=zeros(2,2,np);
figure
Workspace=translate(Workspace,[ 0 -r_foot]);
bestRect=translate(bestRect,[ 0 -r_foot]);
disp("Performing Inverse Kinematics")
for i=1:np
    invang=@(x)Inv_Five_Bar_Ang_Rigid_Foot(x,Walk_Cycle_x(i),Walk_Cycle_y(i),LA,LB,LC,LC1,LD,LN);
    angles=fsolve(invang,q0,options); 
    qA(i)=angles(1);
    qD(i)=angles(4);
    q0=angles;
    Jacobian(:,:,i)=Five_bar_Jacobian_Rigid_Foot(LA,LB,LC,LC1,LD,LN,qA(i),qD(i));
        [E,BC,Bo,Co] = Five_bar_position_rigid_foot(LA,LB,LC,LC1,LD,LN,qA(i),qD(i));
        Cycle_Poses(:,:,i)=[Ao Bo BC E Co Do];
        F(:,i)=E-[0; r_foot]; %Part of foot in contact with ground
        BC_vec(:,i) = BC;
        plot(Workspace,'LineStyle',"none", 'FaceColor',"#4169E1");
        hold on         
        plot(bestRect, 'FaceColor','#6495ED', 'FaceAlpha', 1,'LineStyle',"none");
        plot(F(1,1:i),F(2,1:i),'-','LineWidth',2,'Color',"	#E1C16E")
        plot(Cycle_Poses(1,:,i),Cycle_Poses(2,:,i),'k','LineWidth',2)
        rectangle('Position',[E(1)-r_foot E(2)-r_foot 2*r_foot 2*r_foot],'Curvature',[1 1],'LineWidth',2)
        hold off
        axis([-.1 .4 -.4 .1])
        axis equal
        title('Short Configuration Walk Cycle')
        legend('Reachable workspace','Longest rectangle of 6cm height','Foot-Path')
        drawnow
end
clear angles; clear invang; clear q0; clear Ao; clear Bo; clear BC;
clear Co; clear Do;
%% Speed, Torque, Force Calculations
dqA=([diff(qA); qA(1)-qA(end)]/dt)';
dqD=([diff(qD); qD(1)-qD(end)]/dt)';
qA=qA';
qD=qD';

A_rpm=abs(Gear_Ratio*dqA*60/(2*pi));
D_rpm=abs(Gear_Ratio*dqD*60/(2*pi));

A_I=polyval(Current_fit,A_rpm);
D_I=polyval(Current_fit,D_rpm);

Torque_A=(K_i.*A_I).*sign(dqA);
Torque_D=(K_i.*D_I).*sign(dqD);
R=zeros(np,2);
for j=1:np
R(j,:) = (Jacobian(:,:,j).')\[Gear_Ratio*Torque_A(i); Gear_Ratio*Torque_D(i)];
end

Max_Pushing_Force=max(R(1:stride_point_end,1));
Average_Pushing_Force=mean(R(1:stride_point_end,1));

%% Plotting Postitions, Velocities and Forces
figure
kin=tiledlayout(3,1);
title(kin,PlotTitle)
% Positions
nexttile
plot(t,qA,'LineWidth',3)
hold on
plot(t,qD,'LineWidth',3)
axis tight
title('Angular Position')
xlabel('time (s)')
ylabel('angle (rad)')
xline(t(stride_point_end),'--','LineWidth',3)
legend('qA','qD','End of Stride')

% Velocities
nexttile
plot(t,dqA,'LineWidth',3)
hold on
plot(t,dqD,'LineWidth',3)
axis tight
title('Angular Velocities')
xlabel('time (s)')
ylabel('\omega (rad/s)')
xline(t(stride_point_end),'--','LineWidth',3)
legend('dqA','dqD','End of Stride')


% Forces
nexttile
plot(t(1:end),R(1:end,1),'LineWidth',3)
hold on
plot(t(1:end),R(1:end,2),'LineWidth',3)
axis tight
title('Pushing Forces')
xlabel('time (s)')
ylabel('Force (N)')
xline(t(stride_point_end),'--','LineWidth',3)
legend('Rx','Ry','End of Stride')

%% saving data for export
clear i; clear j;
if ~isempty(File_Name)
save(File_Name)
end