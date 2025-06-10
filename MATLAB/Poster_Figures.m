clear; close all; clc;
workspacecolor="#4169E1";
rectanglecolor='#6495ED';
pathcolor="#E1C16E";
% Loading data
addpath("Walk_Cycles")
load('cycle_shortv2.mat','R','t','stride_point_end','Workspace','bestRect','Cycle_Poses','F','E','r_foot','dqA','dqD')
R_short=R;
t_short=t;
end_p_short=stride_point_end;
workspace_short=Workspace;
rectshort=bestRect;
foot_short=F;
Cycle_Poses_short=Cycle_Poses;
E_short=E;
dqA_short=dqA;
dqD_short=dqD;
load('cycle_ADv2.mat','R','t','stride_point_end','Workspace','bestRect','Cycle_Poses','F','E')
R_AD=R;
t_AD=t;
end_p_AD=stride_point_end;
workspace_AD=Workspace;
rectAD=bestRect;
foot_AD=F;
Cycle_Poses_AD=Cycle_Poses;
E_AD=E;
dqA_AD=dqA;
dqD_AD=dqD;
load('cycle_BCv2.mat','R','t','stride_point_end','Workspace','bestRect','Cycle_Poses','F','E')
R_BC=R;
t_BC=t;
end_p_BC=stride_point_end;
workspace_BC=Workspace;
rectBC=bestRect;
foot_BC=F;
Cycle_Poses_BC=Cycle_Poses;
E_BC=E;
dqA_BC=dqA;
dqD_BC=dqD;
load('cycle_LNv2.mat','R','t','stride_point_end','bestRect','Cycle_Poses','F','E','Workspace')
R_LN=R;
t_LN=t;
end_p_LN=stride_point_end;
workspace_LN=Workspace;
rectLN=bestRect;
foot_LN=F;
Cycle_Poses_LN=Cycle_Poses;
E_LN=E;
dqA_LN=dqA;
dqD_LN=dqD;
 clear R t stride_point_end Workspace bestRect F E;

 %Plotting forces
figure
 plot(t_short(1:end_p_short),R_short(1:end_p_short,1),'LineWidth',2)
 hold on
 plot(t_AD(1:end_p_AD),R_AD(1:end_p_AD,1),'LineWidth',2)
 plot(t_BC(1:end_p_BC),R_BC(1:end_p_BC,1),'LineWidth',2)
 plot(t_LN(1:end_p_LN),R_LN(1:end_p_LN,1),'LineWidth',2)
 axis tight
 title('Stride Pushing Forces')
 subtitle('Foot Velocity .2 m/s')
 xlabel('Time (s)')
 ylabel('Pushing Force (N)')
 legend('Short Configuration',"Elongated AD", "Elongated BC","Elongated LN",'Location','southwest')

 %Plotting Velocities
figure
dq=tiledlayout(2,1);
title(dq,'Angular Velocities')
subtitle(dq,'Foot Velocity = .2m/s')
nexttile
plot(t_short(1:end_p_short),dqA_short(1:end_p_short),'LineWidth',3)
hold on
plot(t_AD(1:end_p_AD),dqA_AD(1:end_p_AD),'LineWidth',3)
plot(t_BC(1:end_p_BC),dqA_BC(1:end_p_BC),'LineWidth',3)
plot(t_LN(1:end_p_LN),dqA_LN(1:end_p_LN),'LineWidth',3)
axis tight
xlabel('Time (s)')
ylabel('\omega (rad/s)')
legend('Short Configuration',"Elongated AD", "Elongated BC","Elongated LN",'Location','northwest')
title('qA''')
nexttile
plot(t_short(1:end_p_short),dqD_short(1:end_p_short),'LineWidth',3)
hold on
plot(t_AD(1:end_p_AD),dqD_AD(1:end_p_AD),'LineWidth',3)
plot(t_BC(1:end_p_BC),dqD_BC(1:end_p_BC),'LineWidth',3)
plot(t_LN(1:end_p_LN),dqD_LN(1:end_p_LN),'LineWidth',3)
axis tight
xlabel('Time (s)')
ylabel('\omega (rad/s)')
legend('Short Configuration',"Elongated AD", "Elongated BC","Elongated LN",'Location','northwest')
title('qD''')

 %% Poses and Workspace
 figure
 WS=tiledlayout(2,2);
title(WS,'Stride Comparison')
 nexttile
 plot(workspace_short,'LineStyle',"none", 'FaceColor',workspacecolor,'FaceAlpha',1);
 hold on
  plot(rectshort,'LineStyle',"none", 'FaceColor',rectanglecolor,'FaceAlpha',1);

 plot(foot_short(1,:),foot_short(2,:),'.','MarkerSize',5,"Color",pathcolor)
  plot(Cycle_Poses_short(1,:,end),Cycle_Poses_short(2,:,end),'k','LineWidth',2)
 rectangle('Position',[E_short(1)-r_foot E_short(2)-r_foot 2*r_foot 2*r_foot],'Curvature',[1 1],'LineWidth',2)
 axis([-.1 .4 -.4 .1])
 axis equal
 title('Short Configuration')
 xlabel('x (m)')
 ylabel('y (m)')

nexttile
plot(workspace_AD,'LineStyle',"none", 'FaceColor',workspacecolor,'FaceAlpha',1);
hold on
plot(rectAD,'LineStyle',"none", 'FaceColor',rectanglecolor,'FaceAlpha',1);

plot(foot_AD(1,:),foot_AD(2,:),'.','MarkerSize',5,"Color",pathcolor)
plot(Cycle_Poses_AD(1,:,end),Cycle_Poses_AD(2,:,end),'k','LineWidth',2)
rectangle('Position',[E_AD(1)-r_foot E_AD(2)-r_foot 2*r_foot 2*r_foot],'Curvature',[1 1],'LineWidth',2)
axis([-.1 .4 -.4 .1])
axis equal
title('Elongated AD')
 xlabel('x (m)')
 ylabel('y (m)')

nexttile
plot(workspace_BC,'LineStyle',"none", 'FaceColor',workspacecolor,'FaceAlpha',1);
hold on
plot(rectBC,'LineStyle',"none", 'FaceColor',rectanglecolor,'FaceAlpha',1);
plot(foot_BC(1,:),foot_BC(2,:),'.','MarkerSize',5,"Color",pathcolor)
plot(Cycle_Poses_BC(1,:,end),Cycle_Poses_BC(2,:,end),'k','LineWidth',2)

rectangle('Position',[E_BC(1)-r_foot E_BC(2)-r_foot 2*r_foot 2*r_foot],'Curvature',[1 1],'LineWidth',2)
axis([-.1 .4 -.4 .1])
axis equal
title('Elongated BC')
 xlabel('x (m)')
 ylabel('y (m)')

nexttile
plot(workspace_LN,'LineStyle',"none", 'FaceColor',workspacecolor,'FaceAlpha',1);
hold on
plot(rectLN,'LineStyle',"none", 'FaceColor',rectanglecolor,'FaceAlpha',1);
plot(foot_LN(1,:),foot_LN(2,:),'.','MarkerSize',5,"Color",pathcolor)
plot(Cycle_Poses_LN(1,:,end),Cycle_Poses_LN(2,:,end),'k','LineWidth',2)

rectangle('Position',[E_LN(1)-r_foot E_LN(2)-r_foot 2*r_foot 2*r_foot],'Curvature',[1 1],'LineWidth',2)
axis([-.1 .4 -.4 .1])
axis equal
title('Elongated LN')
 xlabel('x (m)')
 ylabel('y (m)')
