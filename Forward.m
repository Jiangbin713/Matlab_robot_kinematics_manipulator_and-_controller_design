close all
clear all
%Forword kinematic solution

%% Loading solutions
%cd('C:\Users\jiang\Desktop\CVML\Sem1-Space Robotics and Autonomy\coursework DEC 5 WED');
inverse = load ('inverse_solutions.mat');
Theta_1 = inverse.solution(1,:);
Theta_2 = inverse.solution(2,:);
Theta_3 = inverse.solution(3,:);
Theta_4 = inverse.solution(4,:);
%% Forword solution one

A01 = [ cosd(Theta_1(1))  0    -sind(Theta_1(1))    0;
        sind(Theta_1(1))  0     cosd(Theta_1(1))    0;
        0                -1     0                   0;
        0                 0     0                   1;];
 
A12 = [ cosd(Theta_1(2))  -sind(Theta_1(2))   0     0.5*cosd(Theta_1(2));
        sind(Theta_1(2))   cosd(Theta_1(2))   0     0.5*sind(Theta_1(2));
        0                  0                  1     0.25;
        0                  0                  0     1;];  
    
A23 = [ cosd(Theta_1(3))  0    sind(Theta_1(3))    0;
        sind(Theta_1(3))  0   -cosd(Theta_1(3))    0;
        0                 1     0                  0;
        0                 0     0                  1;];    

A34 = [ cosd(Theta_1(4))  0   -sind(Theta_1(4))    0;
        sind(Theta_1(4))  0    cosd(Theta_1(4))    0;
        0                -1     0                  1;
        0                 0     0                  1;];  

A45 = [ cosd(Theta_1(5))  0    sind(Theta_1(5))    0;
        sind(Theta_1(5))  0   -cosd(Theta_1(5))    0;
        0                 1     0                  0;
        0                 0     0                  1;];    
    
A56 = [ cosd(Theta_1(6))  -sind(Theta_1(6))   0     0;
        sind(Theta_1(6))   cosd(Theta_1(6))   0     0;
        0                  0                  1     0.5;
        0                  0                  0     1;];  
 
F1 = A01*A12*A23*A34*A45*A56;
    
%% Forword solution two

A01 = [ cosd(Theta_2(1))  0    -sind(Theta_2(1))    0;
        sind(Theta_2(1))  0     cosd(Theta_2(1))    0;
        0                -1     0                   0;
        0                 0     0                   1;];
 
A12 = [ cosd(Theta_2(2))  -sind(Theta_2(2))   0     0.5*cosd(Theta_2(2));
        sind(Theta_2(2))   cosd(Theta_2(2))   0     0.5*sind(Theta_2(2));
        0                  0                  1     0.25;
        0                  0                  0     1;];  
    
A23 = [ cosd(Theta_2(3))  0    sind(Theta_2(3))    0;
        sind(Theta_2(3))  0   -cosd(Theta_2(3))    0;
        0                 1     0                  0;
        0                 0     0                  1;];    

A34 = [ cosd(Theta_2(4))  0   -sind(Theta_2(4))    0;
        sind(Theta_2(4))  0    cosd(Theta_2(4))    0;
        0                -1     0                  1;
        0                 0     0                  1;];  

A45 = [ cosd(Theta_2(5))  0    sind(Theta_2(5))    0;
        sind(Theta_2(5))  0   -cosd(Theta_2(5))    0;
        0                 1     0                  0;
        0                 0     0                  1;];    
    
A56 = [ cosd(Theta_2(6))  -sind(Theta_2(6))   0     0;
        sind(Theta_2(6))   cosd(Theta_2(6))   0     0;
        0                  0                  1     0.5;
        0                  0                  0     1;];  
 
F2 = A01*A12*A23*A34*A45*A56;
    

%% Forword solution three

A01 = [ cosd(Theta_3(1))  0    -sind(Theta_3(1))    0;
        sind(Theta_3(1))  0     cosd(Theta_3(1))    0;
        0                -1     0                   0;
        0                 0     0                   1;];
 
A12 = [ cosd(Theta_3(2))  -sind(Theta_3(2))   0     0.5*cosd(Theta_3(2));
        sind(Theta_3(2))   cosd(Theta_3(2))   0     0.5*sind(Theta_3(2));
        0                  0                  1     0.25;
        0                  0                  0     1;];  
    
A23 = [ cosd(Theta_3(3))  0    sind(Theta_3(3))    0;
        sind(Theta_3(3))  0   -cosd(Theta_3(3))    0;
        0                 1     0                  0;
        0                 0     0                  1;];    

A34 = [ cosd(Theta_3(4))  0   -sind(Theta_3(4))    0;
        sind(Theta_3(4))  0    cosd(Theta_3(4))    0;
        0                -1     0                  1;
        0                 0     0                  1;];  

A45 = [ cosd(Theta_3(5))  0    sind(Theta_3(5))    0;
        sind(Theta_3(5))  0   -cosd(Theta_3(5))    0;
        0                 1     0                  0;
        0                 0     0                  1;];    
    
A56 = [ cosd(Theta_3(6))  -sind(Theta_3(6))   0     0;
        sind(Theta_3(6))   cosd(Theta_3(6))   0     0;
        0                  0                  1     0.5;
        0                  0                  0     1;];  
 
F3 = A01*A12*A23*A34*A45*A56;

%% Forword solution four

A01 = [ cosd(Theta_4(1))  0    -sind(Theta_4(1))    0;
        sind(Theta_4(1))  0     cosd(Theta_4(1))    0;
        0                -1     0                   0;
        0                 0     0                   1;];
 
A12 = [ cosd(Theta_4(2))  -sind(Theta_4(2))   0     0.5*cosd(Theta_4(2));
        sind(Theta_4(2))   cosd(Theta_4(2))   0     0.5*sind(Theta_4(2));
        0                  0                  1     0.25;
        0                  0                  0     1;];  
    
A23 = [ cosd(Theta_4(3))  0    sind(Theta_4(3))    0;
        sind(Theta_4(3))  0   -cosd(Theta_4(3))    0;
        0                 1     0                  0;
        0                 0     0                  1;];    

A34 = [ cosd(Theta_4(4))  0   -sind(Theta_4(4))    0;
        sind(Theta_4(4))  0    cosd(Theta_4(4))    0;
        0                -1     0                  1;
        0                 0     0                  1;];  

A45 = [ cosd(Theta_4(5))  0    sind(Theta_4(5))    0;
        sind(Theta_4(5))  0   -cosd(Theta_4(5))    0;
        0                 1     0                  0;
        0                 0     0                  1;];    
    
A56 = [ cosd(Theta_4(6))  -sind(Theta_4(6))   0     0;
        sind(Theta_4(6))   cosd(Theta_4(6))   0     0;
        0                  0                  1     0.5;
        0                  0                  0     1;];  
 
F4 = A01*A12*A23*A34*A45*A56;

%% Result
forward = {F1 , F2 , F3, F4};
for i= 1 : 1: 4
fprintf('Forward kinematics result %d : \n', i ) 
disp(forward{i})
end