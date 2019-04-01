% YOU SHOULD NOT CHANGE THIS FILE
% make sure the robotics toolbox is installed
% to run question you type homework2(questionNo)

function homework2(questionNo)

    if nargin < 1
        error('You did not enter a number');
    end
    
    saluki_rob = initRobot('saluki_{rob}')
    q0 = [0 0 0 -pi/2];
    xyzballPos = [0.3 0.5 -0.5]';
    
    sphereCtr = [0.5;0.0;0];
    sphereR = 0.2;
    
    
    if questionNo == 1
        close all;
        %You need to implement Prob1 in a different file in same folder
        Prob1(saluki_rob)
    end
    
    if questionNo == 2
        close all;
        xyzPos = Prob2(saluki_rob, q0);
    end
    
    if questionNo == 3
        close all; 
        saluki_rob.plot(q0,'jointdiam',2)
        hold on;
        drawBall(xyzballPos,0.1,[0 0 1])
        
        %You need to implement Prob3 in a different file in same folder
        q_ball = Prob3(saluki_rob, q0, xyzballPos)
        view([50,25])
        t = [0:0.025:2]';
        traj = jtraj(q0, q_ball, t);
        %saluki_rob.plot(traj, 'movie', 'prob3.mp4' );
        saluki_rob.plot(traj, 'movie', 'prob3.avi');
    end
    
    if questionNo == 4
        close all;
        saluki_rob.plot(q0,'jointdiam',1)
        hold on;
        drawBall(sphereCtr,sphereR,[1 0 0])
        view([90,18])
        
        q1 = [0.5 -0.95 0 -0.5];
        q2 = [-0.9, -2.5, -1.45, -0.35]
        %q2 = [-1 -3 -1 -.5]
        
         %You need to implement Prob4 in a different file in same folder
        isThereCollision = Prob4(saluki_rob, q1, q2, sphereCtr, sphereR);
        %print('There is a collision and my function returns: ', int2str(isThereCollision));
        if isThereCollision == 1
            disp('There is a collision');
        else
            disp('There is no collision');
        end
        
         t = [0:0.025:2]';
         traj = jtraj(q1, q2, t);
         saluki_rob.plot(traj)
    end
    
    if questionNo == 5
        close all;
        saluki_rob.plot(q0,'jointdiam',1)
        hold on;
        drawBall(sphereCtr,sphereR,[1 0 0])
        view([90,18])
        
        qf = [-0.9, -2.5, -1.45, -0.35];
        
        getMilestones = Prob5(saluki_rob,sphereCtr,sphereR,q0,qf);
        traj = interpMilestones(getMilestones);
        saluki_rob.plot(traj)
    end
    
    if questionNo == 6
        close all;
        saluki_rob.plot(q0,'jointdiam',1)
        hold on;
        drawBall(sphereCtr,sphereR,[1 0 0])
        view([90,18])
        
        qf = [-0.9, -2.5, -1.45, -0.35];
        
        getMilestones = Prob6(saluki_rob,sphereCtr,sphereR,q0,qf);
        traj = interpMilestones(getMilestones);
        saluki_rob.plot(traj)
    end
        
    function rob = initRobot(name)
    
        L(1) = Link([0 0 0 pi/2]);
        L(2) = Link([0 0 0 -pi/2]);
        L(3) = Link([0 0.5 0 -pi/2]);
        L(4) = Link([0 0 0.5 pi/2]);
        rob = SerialLink(L, 'name', name)
        
    end

    function drawBall(pos,d,clr)

		[X,Y,Z] = sphere;
		X=X*d+pos(1);
		Y=Y*d+pos(2);
		Z=Z*d+pos(3);
		hSurface = surf(X,Y,Z);
        hold on 
        set(hSurface,'FaceColor',clr, ...
      'FaceAlpha',0.5,'FaceLighting','gouraud','EdgeColor','none')

    end

    function traj = interpMilestones(getmilestones)
        
        diff = 0.05;
        traj = [];
        
        for i=2:size(getmilestones,1)
            
            delta = getmilestones(i,:) - getmilestones(i-1,:);
            maxVal = max(floor(norm(delta) / diff),1);
            vector = linspace(0,1,maxVal);
            leg = repmat(delta',1,maxVal) .* repmat(vector,size(delta,2),1) + repmat(getmilestones(i-1,:)',1,maxVal);
            traj = [traj;leg'];
            
        end
    end
end