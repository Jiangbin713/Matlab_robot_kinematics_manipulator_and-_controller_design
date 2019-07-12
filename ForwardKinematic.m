function [X,Y,Z]=ForwardKinematic(theta1,theta2,theta3,theta4,theta5,theta6)

A01 = [ cosd(theta1)  0    -sind(theta1)    0;
        sind(theta1)  0     cosd(theta1)    0;
        0                -1     0                   0;
        0                 0     0                   1;];
 
A12 = [ cosd(theta2)  -sind(theta2)   0     0.5*cosd(theta2);
        sind(theta2)   cosd(theta2)   0     0.5*sind(theta2);
        0                  0                  1     0.25;
        0                  0                  0     1;];  
    
A23 = [ cosd(theta3)  0    sind(theta3)    0;
        sind(theta3)  0   -cosd(theta3)    0;
        0                 1     0                  0;
        0                 0     0                  1;];    

A34 = [ cosd(theta4)  0   -sind(theta4)    0;
        sind(theta4)  0    cosd(theta4)    0;
        0                -1     0                  1;
        0                 0     0                  1;];  

A45 = [ cosd(theta5)  0    sind(theta5)    0;
        sind(theta5)  0   -cosd(theta5)    0;
        0                 1     0                  0;
        0                 0     0                  1;];    
    
A56 = [ cosd(theta6)  -sind(theta6)   0     0;
        sind(theta6)   cosd(theta6)   0     0;
        0                  0                  1     0.5;
        0                  0                  0     1;];  
 
F1 = A01*A12*A23*A34*A45*A56;

%% Result
X = F1(1,4);
Y = F1(2,4);
Z = F1(3,4);