%
% This function will generate a collision free path from q0 to xyzPoz using
% the RRT* method
% 
% input: rob - seriallink object for the manipulator
%        sphereCtr - 3x1 position of center of sphere
%        sphereR - radius of sphere
%        q0 - 1x4 vector for the pose
%        qf - 1x4 vector for final pose
% output: milestones - nx4 matrix of milestones. There should no be a
% collision in a strate-line interpolated path through the milestones. The
% first milestone should be q0 and the last milestone should place end
% effector in qf

function milestones = Prob6(rob, sphereCtr, sphereR, q0,qf)
    
    startTrans = rob.fkine(q0);
    endTrans = rob.fkine(qf);
    
    startPoint = startTrans.t;
    endPoint = endTrans.t;
    
    %allPoints = startPoint;
    %allPoints = [allPoints endPoint];
    
    % draw the goal point as a blue sphere
    drawBall(endPoint,0.05,[1 0 0]);
    
    %x_max = 1;
    %y_max = 1;
    %z_max = 1;

    EPS = .15;
    numNodes = 1000;        

    q_start.coord = startPoint;
    q_start.cost = 0;
    q_start.parent = 0;
    q_goal.coord = endPoint;
    q_goal.cost = 0;

    nodes(1) = q_start;
    figure(1)
    
    cx = sphereCtr(1);
    cy = sphereCtr(2);
    cz = sphereCtr(3);
    
    %rng(3);

    for i = 1:1:numNodes
        
        q_rand = [-1+2*rand(1) -1+2*rand(1) -1+2*rand(1)];
        %plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  [0 0.4470 0.7410])
        
        x = q_rand(1);
        y = q_rand(2);
        z = q_rand(3);

        while ~((x-cx)^2 + (y-cy)^2 + (z-cz)^2 > sphereR^2) || (norm(q_rand - [0 0 0]) > 1)

            q_rand = [-1+2*rand(1) -1+2*rand(1) -1+2*rand(1)];
            x = q_rand(1);
            y = q_rand(2);
            z = q_rand(3);
        end
        
        %if ~((x-cx)^2 + (y-cy)^2 + (z-cz)^2 > sphereR^2)
        %    disp("this should be impossible");
        %end
        
        
        %plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  [0 0.4470 0.7410])
        %plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  [0 0 0])

        % Break if goal node is already reached
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_goal.coord
                break
            end
        end

        % Pick the closest node from existing list to branch out from
        ndist = [];
        for j = 1:1:length(nodes)
            n = nodes(j);
            tmp = dist_3d(n.coord, q_rand);
            ndist = [ndist tmp];
        end
        [val, idx] = min(ndist);
        q_near = nodes(idx);

        q_new.coord = steer3d(q_rand, q_near.coord, val, EPS);
        
         if ~((q_new.coord(1)-cx)^2 + (q_new.coord(2)-cy)^2 + (q_new.coord(3)-cz)^2 > sphereR^2) || (norm(q_rand - [0 0 0]) > 1)
             %q_new.coord = goodval(q_rand. q_near.coord, val, EPS);
             continue;
         end
        
        
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;

        % Within a radius of r, find all existing nodes
        q_nearest = [];
        r = .35;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if (dist_3d(nodes(j).coord, q_new.coord)) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end

        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;

        % Iterate through all nearest neighbors to find alternate lower
        % cost paths

        for k = 1:1:length(q_nearest)
            if q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'g');            
                hold on
            end
        end

        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end

        % Append to nodes
        nodes = [nodes q_new];
    end

    D = [];
    for j = 1:1:length(nodes)
        tmpdist = dist_3d(nodes(j).coord, q_goal.coord);
        D = [D tmpdist];
    end

    % Search backwards from goal to start to find the optimal least cost path
    [val, idx] = min(D);
    q_final = nodes(idx);
    q_goal.parent = idx;
    q_end = q_goal;
    nodes = [nodes q_goal];
    mileStonesC = q_end.coord; % add ending point
    while q_end.parent ~= 0
        start = q_end.parent;
        line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'r', 'LineWidth', 4);
        %mileStonesC = [mileStonesC q_end.coord];
        hold on
        q_end = nodes(start);
        if size(q_end.coord,2) == 3
            mileStonesC = [mileStonesC q_end.coord'];
        end
    end
    
    mileStonesC = [mileStonesC q_end.coord]; % add starting point
    % so now mileStonesC is filled but backwards
    
    mileStonesC = fliplr(mileStonesC);
    % now in start to end order
    
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
