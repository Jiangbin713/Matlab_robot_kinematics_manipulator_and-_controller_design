close all
clear 
%Inverse kinematic solution
%% Link length a_i(m)
a_1 = 0; a_3 = 0; a_4 = 0; a_5 = 0;  a_6 = 0;
a_2 = 0.5;

%% Offset distance d_i(m)
d_1 = 0; d_3 = 0; d_5 = 0;
d_2 = 0.25;
d_4 = 1;
d_6 =0.5;

%%  Giving D-H transform matrix
T06 = [-1/sqrt(2)    0     1/sqrt(2)     1;
       0             -1     0            1;
       1/sqrt(2)      0     1/sqrt(2)    0
       0              0     0            1]; 
   
 n_x = T06(1,1); n_y = T06(2,1); n_z = T06(3,1);
 s_x = T06(1,2); s_y = T06(2,2); s_z = T06(3,2);
 a_x = T06(1,3); a_y = T06(2,3); a_z = T06(3,3);
 p_x = T06(1,4); p_y = T06(2,4); p_z = T06(3,4);
 
p_arm = [p_x; p_y; p_z] - d_6 * [a_x; a_y; a_z]; 
p_x_arm = p_arm(1); p_y_arm = p_arm(2); p_z_arm = p_arm(3);
 %% theta_1  
   theta_1_one = atan2d(p_y_arm, p_x_arm) - atan2d( d_2 , +sqrt( p_x_arm^2 + p_y_arm^2 - d_2^2 )  ); %degree1
   theta_1_two = atan2d(p_y_arm, p_x_arm) - atan2d( d_2 , -sqrt( p_x_arm^2 + p_y_arm^2 - d_2^2 )  ); %degree2
   theta_1_one_r = theta_1_one*2*pi/360; % radian1
   theta_1_two_r = theta_1_two*2*pi/360; % radian2

 
   
 %% theta_2
   %theta_1_one A_1 B_1
   A_1 = cos(theta_1_one_r) * p_x_arm + sin(theta_1_one_r) * p_y_arm;
   B_1 = ( A_1^2+ p_z_arm^2 + a_2^2 -d_4^2 ) / (2*a_2);
   %theta_1_two A_2 B_2
   A_2 = cos(theta_1_two_r) * p_x_arm + sin(theta_1_two_r) * p_y_arm;
   B_2 = ( A_2^2+ p_z_arm^2 + a_2^2 -d_4^2) / (2*a_2);
   
   theta_2_one = atan2d(A_1, p_z_arm) - atan2d(B_1, +sqrt( A_1^2 + p_z_arm^2 -B_1^2) ); %  degree1
   theta_2_two = atan2d(A_1, p_z_arm) - atan2d(B_1, -sqrt( A_1^2 + p_z_arm^2 -B_1^2) ); %  degree2
   
   theta_2_three = atan2d(A_2, p_z_arm) - atan2d(B_2, +sqrt( A_2^2 + p_z_arm^2 -B_2^2) ); %  degree3
   theta_2_four = atan2d(A_2, p_z_arm) - atan2d(B_2, -sqrt( A_2^2 + p_z_arm^2 -B_2^2) ); %  degree4
   
   theta_2_one_r = theta_2_one * 2 * pi /360; %  radian1
   theta_2_two_r = theta_2_two * 2 * pi /360; %  radian2
   theta_2_three_r = theta_2_three * 2 * pi /360; %  radian3
   theta_2_four_r = theta_2_four * 2 * pi /360; %  radian4
   
 %% theta_3
     %  theta_2_one A_1 B_1
     coefficience_y_1 = A_1 - a_2 * cos( theta_2_one_r );
     coefficience_x_1 = p_z_arm + a_2 * sin( theta_2_one_r ) ;
    
     coefficience_y_2 = A_1 - a_2 * cos( theta_2_two_r );
     coefficience_x_2 = p_z_arm + a_2 * sin( theta_2_two_r ) ;
     
     %  theta_2_three A_2 B_2
     coefficience_y_3 = A_2 - a_2 * cos( theta_2_three_r );
     coefficience_x_3 = p_z_arm + a_2 * sin( theta_2_three_r ) ;
  
     coefficience_y_4 = A_2 - a_2 * cos( theta_2_four_r );
     coefficience_x_4 = p_z_arm + a_2 * sin( theta_2_four_r ) ;
     
     theta_3_one = ( atan2d ( coefficience_y_1 , coefficience_x_1 ) ) - theta_2_one; % degree1
     theta_3_two = ( atan2d ( coefficience_y_2 , coefficience_x_2 ) ) - theta_2_two; % degree2
     
     theta_3_three = ( atan2d ( coefficience_y_3 , coefficience_x_3 ) ) - theta_2_three; % degree3
     theta_3_four = ( atan2d ( coefficience_y_4 , coefficience_x_4 ) ) - theta_2_four; % degree4
     
     theta_3_one_r = theta_3_one * 2 * pi /360; %  radian1
     theta_3_two_r = theta_3_two * 2 * pi /360; %  radian2
     theta_3_three_r = theta_3_three * 2 * pi /360; %  radian3
     theta_3_four_r = theta_3_four * 2 * pi /360; %  radian4
     
  %% theta_4
     
      coefficience_y_1 = -sin(theta_1_one_r) * a_x + cos(theta_1_one_r) * a_y;
      coefficience_y_2 = -sin(theta_1_two_r) * a_x + cos(theta_1_two_r) * a_y;
      
      coefficience_x_1 = cos( theta_2_one_r + theta_3_one_r ) * ( cos(theta_1_one_r) * a_x + sin(theta_1_one_r) * a_y )  ......
                         - sin( theta_2_one_r + theta_3_one_r ) * a_z;
      coefficience_x_2 = cos( theta_2_two_r + theta_3_two_r ) * ( cos(theta_1_one_r) * a_x + sin(theta_1_one_r) * a_y )  ......
                         - sin( theta_2_two_r + theta_3_two_r ) * a_z; 
                     
      coefficience_x_3 = cos( theta_2_three_r + theta_3_three_r ) * ( cos(theta_1_two_r) * a_x + sin(theta_1_two_r) * a_y )  ......
                         - sin( theta_2_three_r + theta_3_three_r ) * a_z;
      coefficience_x_4 = cos( theta_2_four_r + theta_3_four_r ) * ( cos(theta_1_two_r) * a_x + sin(theta_1_two_r) * a_y )  ......
                         - sin( theta_2_four_r + theta_3_four_r ) * a_z;
                     
      theta_4_one = atan2d ( coefficience_y_1 , coefficience_x_1 ); %degree1
      theta_4_two = atan2d ( coefficience_y_1 , coefficience_x_2 ); %degree2
      theta_4_three = atan2d ( coefficience_y_2 , coefficience_x_3 ); %degree3
      theta_4_four = atan2d ( coefficience_y_2 , coefficience_x_4 ); %degree4
      
      theta_4_one_r = theta_4_one * 2 * pi /360; %radian1
      theta_4_two_r = theta_4_two * 2 * pi /360; %radian2
      theta_4_three_r = theta_4_three * 2 * pi /360; %radian3
      theta_4_four_r = theta_4_four * 2 * pi /360; %radian4
      
  %% theta_5
      
coefficience_x_1 = sin( theta_2_one_r + theta_3_one_r ) * ( cos(theta_1_one_r) * a_x + sin(theta_1_one_r) * a_y )......
                   + cos( theta_2_one_r + theta_3_one_r ) * a_z;
               
coefficience_x_2 = sin( theta_2_two_r + theta_3_two_r ) * ( cos(theta_1_one_r) * a_x + sin(theta_1_one_r) * a_y )......
                   + cos( theta_2_two_r + theta_3_two_r ) * a_z; 

coefficience_x_3 = sin( theta_2_three_r + theta_3_three_r ) * ( cos(theta_1_two_r) * a_x + sin(theta_1_two_r) * a_y )......
                   + cos( theta_2_three_r + theta_3_three_r ) * a_z; 

coefficience_x_4 = sin( theta_2_four_r + theta_3_four_r ) * ( cos(theta_1_two_r) * a_x + sin(theta_1_two_r) * a_y )......
                   + cos( theta_2_four_r + theta_3_four_r ) * a_z; 
               
A_1 = cos(theta_1_one_r) * cos( theta_2_one_r + theta_3_one_r ) * a_x...
    + sin(theta_1_one_r) * cos( theta_2_one_r + theta_3_one_r ) * a_y...
    - sin( theta_2_one_r + theta_3_one_r ) * a_z;

A_2 = cos(theta_1_one_r) * cos( theta_2_two_r + theta_3_two_r ) * a_x...
    + sin(theta_1_one_r) * cos( theta_2_two_r + theta_3_two_r ) * a_y...
    - sin( theta_2_two_r + theta_3_two_r ) * a_z;

A_3 = cos(theta_1_two_r) * cos( theta_2_three_r + theta_3_three_r ) * a_x...
    + sin(theta_1_two_r) * cos( theta_2_three_r + theta_3_three_r ) * a_y...
    - sin( theta_2_three_r + theta_3_three_r ) * a_z;

A_4 = cos(theta_1_two_r) * cos( theta_2_four_r + theta_3_four_r ) * a_x...
    + sin(theta_1_two_r) * cos( theta_2_four_r + theta_3_four_r ) * a_y...
    - sin( theta_2_four_r + theta_3_four_r ) * a_z;


B_1 = -sin(theta_1_one_r)*a_x + cos(theta_1_one_r) * a_y;

B_2 = -sin(theta_1_two_r)*a_x + cos(theta_1_two_r) * a_y;

theta_5_one = atan2d( sqrt( A_1^2 + B_1^2 ) , coefficience_x_1 ); %degree1
theta_5_two = atan2d( sqrt( A_2^2 + B_1^2 ) , coefficience_x_2 ); %degree2
theta_5_three = atan2d( sqrt( A_3^2 + B_2^2 ) , coefficience_x_3 ); %degree3
theta_5_four = atan2d( sqrt( A_4^2 + B_2^2 ) , coefficience_x_4 ); %degree4

theta_5_one_r = theta_5_one * 2 * pi /360; %radian1
theta_5_two_r = theta_5_two * 2 * pi /360; %radian2
theta_5_three_r = theta_5_three * 2 * pi /360; %radian3
theta_5_four_r = theta_5_four * 2 * pi /360; %radian4

%% theta_6

coefficient_y_1 = sin( theta_2_one_r + theta_3_one_r ) * ( cos(theta_1_one_r)*s_x + sin(theta_1_one_r)*s_y )...
                  + cos( theta_2_one_r + theta_3_one_r ) * s_z;
coefficient_y_2 = sin( theta_2_two_r + theta_3_two_r ) * ( cos(theta_1_one_r)*s_x + sin(theta_1_one_r)*s_y )...
                  + cos( theta_2_two_r + theta_3_two_r ) * s_z;  
coefficient_y_3 = sin( theta_2_three_r + theta_3_three_r ) * ( cos(theta_1_two_r)*s_x + sin(theta_1_two_r)*s_y )...
                  + cos( theta_2_three_r + theta_3_three_r ) * s_z;
coefficient_y_4 = sin( theta_2_four_r + theta_3_four_r ) * ( cos(theta_1_two_r)*s_x + sin(theta_1_two_r)*s_y )...
                  + cos( theta_2_four_r + theta_3_four_r ) * s_z;  
              
coefficient_x_1 = -( sin( theta_2_one_r + theta_3_one_r ) * ( cos(theta_1_one_r)*n_x + sin(theta_1_one_r)*n_y )...
                    + cos( theta_2_one_r + theta_3_one_r )*n_z );
coefficient_x_2 = -( sin( theta_2_two_r + theta_3_two_r ) * ( cos(theta_1_one_r)*n_x + sin(theta_1_one_r)*n_y )...
                    + cos( theta_2_two_r + theta_3_two_r )*n_z );
coefficient_x_3 = -( sin( theta_2_three_r + theta_3_three_r ) * ( cos(theta_1_two_r)*n_x + sin(theta_1_two_r)*n_y )...
                    + cos( theta_2_three_r + theta_3_three_r )*n_z );
coefficient_x_4 = -( sin( theta_2_four_r + theta_3_four_r ) * ( cos(theta_1_two_r)*n_x + sin(theta_1_two_r)*n_y )...
                    + cos( theta_2_four_r + theta_3_four_r )*n_z );    

theta_6_one = atan2d( coefficient_y_1 , coefficient_x_1  ); %degree1
theta_6_two = atan2d( coefficient_y_2 , coefficient_x_2  ); %degree2
theta_6_three = atan2d( coefficient_y_3 , coefficient_x_3  ); %degree3
theta_6_four = atan2d( coefficient_y_4 , coefficient_x_4  ); %degree4

theta_6_one_r = theta_6_one * 2 * pi /360; %radian1
theta_6_two_r = theta_6_two * 2 * pi /360; %radian2      
theta_6_three_r = theta_6_three * 2 * pi /360; %radian1
theta_6_four_r = theta_6_four * 2 * pi /360; %radian2     

%% results
solution = [ theta_1_one theta_2_one theta_3_one theta_4_one theta_5_one theta_6_one ;
               theta_1_one theta_2_two theta_3_two theta_4_two theta_5_two theta_6_two ;
               theta_1_two theta_2_three theta_3_three theta_4_three theta_5_three theta_6_three ;
               theta_1_two theta_2_four theta_3_four theta_4_four theta_5_four theta_6_four ];

for i = 1 :1 :4
fprintf ('the solution %d (degree) £º', i)
fprintf ('%3.2f ', solution( i, :))
fprintf('\n')
end
save('inverse_solutions', 'solution');