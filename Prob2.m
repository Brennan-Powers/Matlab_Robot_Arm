% 
% Take a q0 position and calculate xyz of the end effector.
% Don't use fkine, find equations.
%
function qangles = Prob2(rob, q0)
    
    pt = rob.fkine(q0);
    disp(pt);
    
    %T1 = rob.base;
    
    T1 = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
    
    for i=1 : rob.n
        T1 = T1 * [ cos(q0(i)) -sin(q0(i))*cos(rob.alpha(i)) sin(q0(i))*sin(rob.alpha(i)) rob.a(i)*cos(q0(i))
                  sin(q0(i)) cos(q0(i))*cos(rob.alpha(i)) -cos(q0(i))*sin(rob.alpha(i)) rob.a(i)*sin(q0(i))
                  0          sin(rob.alpha(i))             cos(rob.alpha(i))            rob.d(i)
                  0          0                             0                            1 ];
              
    end
    
    %T1 = T1 * rob.tool;
    
    T1 = T1 * [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
    
    qangles = T1(1:3,4);
        

    %b = transl(0,0,rob.alpha(1)) * transl(.5,0,0) * transl(0,0,rob.alpha(2)) * transl(.5,0,0) * transl(0,0,rob,alpha(3)) * transl(.5,0,0) * transl(0,0,rob.alpha(4)) * transl(.5,0,0);
    
    %b = transl(0,0,rob.alpha(1)) * transl(.5,0,0);
    
%     b = (transl(0,0,rob.alpha(1)) * transl(0,0,0)); 
%     c = (transl(0,0,rob.alpha(2)) * transl(0,0,0)); 
%     d = (transl(0,0,rob.alpha(3)) * transl(.5,0,0)); 
%     e = (transl(0,0,rob.alpha(4)) * transl(.5,0,0));
%     
%     f = b*c*d*e;
    
    %q0 = [rob.alpha(1) rob.alpha(2) rob.alpha(3) rob.alpha(4)];
    %q0 = [pi/2 pi/2 0 -pi/2];
    rob.plot(q0,'jointdiam',2);
    
    %disp(f);

    %qangles = pt;





end