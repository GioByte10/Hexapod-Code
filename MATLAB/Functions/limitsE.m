function Workspace=limitsE(LA,LB,LC,LC1,LD,LN,n,k,vis)
if nargin < 9
    vis=0;
end
if nargin < 8
    k=.35; 
end
if nargin < 7
    n=200;
end
warning('off','MATLAB:polyshape:repairedBySimplify')
    % initializing vector of angles to search
    q=linspace(0,2*pi,n);
    % Find position of E for each angle
    E_array=zeros(2,n*n);
    for i=1:n
        for j=1:n
            [E] = Five_bar_position_rigid_foot(LA,LB,LC,LC1,LD,LN,q(i),q(j));
            E_array(:,(j-1)*n+i)=E;
        end
    end
    %find boundary
    Bound=boundary(E_array(1,:)',E_array(2,:)',k);
    %make workspace
    Workspace=polyshape(E_array(1,Bound),E_array(2,Bound));
    %Make a plot
    if vis==1
         figure
    plot(Workspace)         

    hold on
    plot(E_array(1,:),E_array(2,:),'k.')    
    plot(E_array(1,Bound),E_array(2,Bound),'r','LineWidth',2);  
    legend('Workspace', 'Foot Postions','Boundary')
    title('Workspace Boundary')
    end

end
