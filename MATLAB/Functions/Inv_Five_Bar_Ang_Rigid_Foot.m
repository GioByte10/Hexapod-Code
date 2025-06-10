function q=Inv_Five_Bar_Ang_Rigid_Foot(X,xE, yE,LA,LB,LC,LC1,LD,LN)
% This function allows you to find qA qB qC & qD for a given foot position
% E

% Inputs
% LA,LB,LC,LD Lengths of links A, B, C, D, Ground Link
% LC1 Length from Co to BC
q(1)=LA*cos(X(1)) + LB*cos(X(2)) - LC1*cos(X(3)) - LD*cos(X(4)) - LN;
q(2)=LA*sin(X(1)) + LB*sin(X(2)) - LC1*sin(X(3)) - LD*sin(X(4));
q(3)=LC*sin(X(3)) + LD*sin(X(4)) - yE;
q(4)=LN + LC*cos(X(3)) + LD*cos(X(4)) - xE;
%outputs q [qA qB qC qD]
end