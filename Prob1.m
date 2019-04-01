%
% This function will take as an input the robot object and plot three poses
% of the robot
% 
% input: rob - seriallink object for the manipulator
%        
% output: empty
% 
function Prob1(rob)
    
    q0 = [0 0 0 0];
    rob.plot(q0,'jointdiam',2);
    pause(5);
    
    q1 = [1 1 1 1];
    rob.plot(q1,'jointdiam',2);
    pause(5);
    
    q2 = [pi/2 .5 pi/2 .5];
    rob.plot(q2,'jointdiam',2);
    pause(5);
    
    q0 = [1 .5 .5 1];
    rob.plot(q0,'jointdiam',2);
    
end
