function [maxLength,bestRect] = WorkspaceRectangle(Workspace,h)
maxLength = 0;
bestRect = [];
k=1;
Lres=300;
    maxL = max(Workspace.Vertices(:,1)) - min(Workspace.Vertices(:,1)); % upper bound on length
    test_lengths = linspace(1e-6, maxL,Lres); % resolution of length test
    n=length(Workspace.Vertices);
for i = 1:n
    x0 = Workspace.Vertices(i,1);
    y0 = Workspace.Vertices(i,2);
    
    % Skip if top-left is not inside polygon
    if ~isinterior(Workspace, x0, y0+h)
        continue;
    end
    % Try growing lengthwise
    for j=k:Lres
        L = test_lengths(j);
        % Define rectangle polygon
        rectX = [x0, x0+L, x0+L, x0];
        rectY = [y0, y0, y0+h, y0+h];
        testRect = polyshape(rectX, rectY);       
        % Check containment
        if abs(area(intersect(testRect, Workspace)) - area(testRect)) < 1e-9
            if L > maxLength
                maxLength = L;
                k=j;
                bestRect = testRect;
            end
        else
            break; % stop growing once it doesn't fit
        end
    end
end