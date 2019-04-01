%
% This function will take as an input the robot object, the initial pose 
% and final pose of the robot with respect to its joints and return a
% boolean to indicate whether there is an obstacle collision. 
% 
% input: rob - seriallink object for the manipulator
%        q0 - initial pose 1x4 vector
%        qf - final pose 1x4 vector
%        sphereCtr - 3x1 position of center of sphere
%        sphereR - radius of sphere
%
% output: isThereCollision - boolean indicating whether there is a
% collision
% 
% Your algorithm should check for collision in n equidistant points
% in a straight line between the end effector in q0 and qf
%
function isThereCollision = Prob4(rob, q0, qf, sphereCtr, sphereR)
    
    % x,y,z = points on line
    % cx,cy,cz = center of sphere
    
    cx = sphereCtr(1);
    cy = sphereCtr(2);
    cz = sphereCtr(3);
    
    T1 = rob.fkine(q0);
    T2 = rob.fkine(qf);
    
    p1 = T1.t;
    p2 = T2.t;
    
    pts = [p1';p2'];
    %line(pts(:,1), pts(:,2), pts(:,3));
    plot3(pts(:,1), pts(:,2), pts(:,3));
    
    % Need to make a line between q0 and qf, divide it into equidistant
    % sections, then get the x,y,z of points along the line/sections.
    % Then run the if below in a loop for all of the points, if any point
    % found not outside sphere, there is collision and can just break,
    % otherwise no collision.

    d = p2 - p1; % d=direction vector from q0 to qf
    dist = norm(p2 - p1); % dist=distance between q0 and qf
    
    step = dist / 100;
    
    isThereCollision = 0;
    
    for i=step : step : dist
        point = (i*d)+p1;
        x = point(1);
        y = point(2);
        z = point(3);

        % true if point x,y,z is not outside the sphere
        if ~((x-cx)^2 + (y-cy)^2 + (z-cz)^2 > sphereR^2)
            isThereCollision = 1;
            return;
            %break;
        end
    end
    
    pause(2);
    
    %isThereCollision = 0;
    
    %pause(2);
    
end

