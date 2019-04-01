%
% This function will take as an input the robot object, the initial pose of
% the robot with respect to its joints, a goalPos position vector and
% return a goal pose.
% 
% input: rob - seriallink object for the manipulator
%        q0 - initial pose 1x4 vector
%        goalPos - goal position 3x1 vector for x, y, z
% output: q_goal - final pose where end effector is in goalPos
% 
% hint: you want to use the inverse kinematiks function in RTB called
% ikine. Note that the ikine function can be used to determine both the
% orientation and position of the end effector. In our case we only care
% about the position (the mask parameter can be used to define this). 
% Use help ikine command to get more info on the function. Also, chapter 8
% in the Corke book is helpful.
%
function q_goal = Prob3(rob, q0, goalPos)
    
    %T = rob.fkine(q0);
    %disp(T);
    %q_goal = T;
    
    %T1 = transl(0,0,1);
    %T2 = transl(goalPos);
    %T = ctraj(T1, T2);
    %q = rob.ikine(T);
    
    %q = [0 0 0 0];
    %T = rob.fkine(q0);
    
    %T2 = transl(0,0,q0(1))*transl(rob.links(3).d,0,0);
    
    %za = asin(goalPos(1,2)/
    
    
    
    
     %T1 = rob.fkine(q0);
%     
     %T2 = T1 * goalPos;
%     
     %T3 = transl(T2);
%     
%     T4 = rob.ikine(T3,'mask',[1,1,0,1,1,0]);
%     
%     q_goal = T4;

    %T0 = rob.fkine(q0);
    %m = [1 1 0 1 1 0];
    %q = rob.ikine(T0, goalPos, m);
    %q = rob.ikcon(T3);
    
    T4 = transl(goalPos);
    
    %q = rob.ikine(T4);

    %q = rob.ikcon(T4);
    
    q = rob.ikine(T4, 'mask', [1 1 1 0 0 0]);
    
    q_goal = q;
    
    %pause(2);
    
end


