function [xx, yy, d, total_length] = WalkCycle(np, xc, yc, h, l, r)
 yc=yc+r;
% Walk_Cycle Computes equally spaced points around a custom capsule-like shape
%   Top: upper half of an ellipse
%   Bottom: two quarter-circle arcs + straight line
%
% Inputs:
%   np - number of equally spaced perimeter points
%   xc, yc - center of the top ellipse
%   h - total height of the shape
%   l - total width (ellipse width)
%   r - radius of bottom arcs
%
% Outputs:
%   xx, yy - coordinates of equally spaced points
%   d - approximate spacing between points

    % Semi-axes of the ellipse
    a = l / 2;
    b = h - r;  % subtract two arc radii from height for ellipse part

    % High-resolution for smooth perimeter
    n = 5000;

    % === 1. Bottom straight line (left to right) ===
    x_line = linspace(xc - (l - 2*r)/2, xc + (l - 2*r)/2, n);
    y_line = (yc-r) * ones(1, n);
    x_line(end)=[];
    y_line(end)=[];
    % === 2. Right arc (bottom to top, counterclockwise) ===
    cx_right = xc + a - r;
    cy_arc = yc ;
    t_arc1 = linspace(3*pi/2, 2*pi, n);
    x_arc1 = cx_right + r * cos(t_arc1);
    y_arc1 = cy_arc + r * sin(t_arc1);
    x_arc1(end)=[];
    y_arc1(end)=[];

    % === 3. Top half ellipse (right to left) ===
    t_ellipse = linspace(0, pi, n);  % from left to right
    x_ellipse = xc + a * cos(t_ellipse);
    y_ellipse = yc + b * sin(t_ellipse);
    x_ellipse(end)=[];
    y_ellipse(end)=[];

    % === 4. Left arc (top to bottom, counter-clockwise) ===
    cx_left = xc - a + r;
    t_arc2 = linspace(pi, 3*pi/2, n);
    x_arc2 = cx_left + r * cos(t_arc2);
    y_arc2 = cy_arc + r * sin(t_arc2);
    x_arc2(end)=[];
    y_arc2(end)=[];

    % === Combine all parts ===
    x_full =[x_line, x_arc1, x_ellipse, x_arc2];
    y_full = [ y_line, y_arc1, y_ellipse,y_arc2];

    % === Arc length parameterization ===
    dx = diff(x_full);
    dy = diff(y_full);
    ds = sqrt(dx.^2 + dy.^2);
    s = [0, cumsum(ds)];
    total_length = s(end);

    % === Interpolate equally spaced points ===
    s_eq = linspace(0, total_length, np + 1);
    s_eq(end) = [];  % remove duplicated point at end
    xx = interp1(s, x_full, s_eq);
    yy = interp1(s, y_full, s_eq);

    % === Approximate spacing between points ===
    d = total_length / np;
end