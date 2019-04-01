%
% This function will generate a collision free path from q0 to xyzPoz using
% the PRM method
% 
% input: rob - seriallink object for the manipulator
%        sphereCtr - 3x1 position of center of sphere
%        sphereR - radius of sphere
%        q0 - 1x4 vector for the pose
%        xyzPos - 3x1 position of the end-effector
% output: milestones - nx4 matrix of milestones. There should no be a
% collision in a strate-line interpolated path through the milestones. The
% first milestone should be q0 and the last milestone should place end
% effector in qf

function milestones = Prob5(rob, sphereCtr, sphereR, q0,qf)
    
    startTrans = rob.fkine(q0);
    endTrans = rob.fkine(qf);
    
    startPoint = startTrans.t;
    endPoint = endTrans.t;
    
    allPoints = startPoint;
    allPoints = [allPoints endPoint];
    
    % draw the goal point as a blue sphere
    drawBall(endPoint,0.05,[1 0 0]);
    
    for i=1 : 5000
        r = -1 + (1+1) * rand(3,1);
        allPoints = [allPoints r];
    end
    
    cx = sphereCtr(1);
    cy = sphereCtr(2);
    cz = sphereCtr(3);
    
    % remove points that are inside sphere
    i=3;
    while i <= size(allPoints, 2)
        x = allPoints(1,i);
        y = allPoints(2,i);
        z = allPoints(3,i);
        % true if collision
        if ~((x-cx)^2 + (y-cy)^2 + (z-cz)^2 > sphereR^2)
            allPoints(:,i) = [];
            i = i-1;
        end
        i = i+1;
    end
    
    % remove points that are beyond the arms reach
    i = 3;
    while i <= size(allPoints, 2)
        if norm(allPoints(:,i) - [0;0;0]) > 1
            allPoints(:,i) = [];
            i = i-1;
        end
        i = i+1;
    end
    
    allPoints = unique(allPoints.','rows','stable').';
    
    nextTo = zeros(size(allPoints,2),size(allPoints,2));
    
    w = .2; % max distance between 2 points
    
    collision = 0; % 0 for no collision, 1 for collision
    
    for i=1 : size(allPoints,2)
        %iterate through points along columns
        
        for j=1 : size(allPoints,2)
            %compare point to all other rows
            %if distance between them is less than w,
            %make nextTo(i,j)=1
            if norm(allPoints(:,j) - allPoints(:,i)) <= w
                
                xpts = linspace(allPoints(1,j), allPoints(1,i), 20);
                ypts = linspace(allPoints(2,j), allPoints(2,i), 20);
                zpts = linspace(allPoints(3,j), allPoints(3,i), 20);
                
                for k=1 : 20
                    if ~((xpts(k)-cx)^2 + (ypts(k)-cy)^2 + (zpts(k)-cz)^2 > sphereR^2)
                        collision=1;
                        break;
                    end
                end
                        
                if collision == 0
                    nextTo(i,j) = 1;
                end
                collision = 0;
                % code to plot all of the edges in the prm
%                 b1 = allPoints(:,i);
%                 b2 = allPoints(:,j);
%                 v1 = [b1(1),b1(2),b1(3)];
%                 v2 = [b2(1),b2(2),b2(3)];
%                 v = [v2;v1];
%                 plot3(v(:,1), v(:,2), v(:,3), 'b');
            end
        end
    end
    
    % Now need to figure out which nodes to go through to get from start to
    % goal. 1=link, 0=no link, on nextTo matrix.
    
    % depth first search? A* ?
    
    g = PGraph(3);
    %nodelist = [];
    %edgelist = [];
    for i=1 : size(allPoints,2)
        g.add_node(allPoints(:,i));
    end
    
    for i=1 : size(allPoints,2)
        for j=1 : size(allPoints,2)
            if i == j
                %nada
            elseif nextTo(i,j) == 1
                g.add_edge(i,j);
            end
        end
    end
    
    h = g.Astar(1,2);
    mileStonesC = g.vertexlist(:,1);
    for i=2 : size(h,2)
        mileStonesC = [mileStonesC g.vertexlist(:,h(i))];
    end
    
    line(mileStonesC(1,:), mileStonesC(2,:), mileStonesC(3,:));
    
    milestones = q0';
    for i=2 : size(mileStonesC,2)
        milestones = [milestones (rob.ikine(transl(mileStonesC(:,i)),'mask',[1 1 1 0 0 0]))' ];
    end
    
    milestones = milestones';
    
    pause(2);
end

    function drawBall(pos,d,clr)

		[X,Y,Z] = sphere;
		X=X*d+pos(1);
		Y=Y*d+pos(2);
		Z=Z*d+pos(3);
		hSurface = surf(X,Y,Z);
        hold on 
        set(hSurface,'FaceColor',clr, ...
      'FaceAlpha',0.5,'FaceLighting','gouraud','EdgeColor','blue')

    end

