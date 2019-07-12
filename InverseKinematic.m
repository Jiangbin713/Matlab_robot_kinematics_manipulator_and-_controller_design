function [theta1,theta2,theta3,theta4,theta5,theta6]=InverseKinematic(T)

%% Link length a_i(m)
a_1 = 0; a_3 = 0; a_4 = 0; a_5 = 0;  a_6 = 0;
a_2 = 0.5;
%% Offset distance d_i(m)
d_1 = 0; d_3 = 0; d_5 = 0;
d_2 = 0.25;
d_4 = 1;
d_6 =0.5;
%% Giving T
n_x = T(1,1); n_y = T(2,1); n_z = T(3,1);
s_x = T(1,2); s_y = T(2,2); s_z = T(3,2);
a_x = T(1,3); a_y = T(2,3); a_z = T(3,3);
p_x = T(1,4); p_y = T(2,4); p_z = T(3,4);
 
p_arm = [p_x; p_y; p_z] - d_6 * [a_x; a_y; a_z]; 
p_x_arm = p_arm(1); p_y_arm = p_arm(2); p_z_arm = p_arm(3);
%% theta_1  
   theta_1_one = atan2d(p_y_arm, p_x_arm) - atan2d( d_2 , +sqrt( p_x_arm^2 + p_y_arm^2 - d_2^2 )  ); %degree1
   theta_1_one_r = theta_1_one*2*pi/360; % radian1
%% theta_2
   %theta_1_one A_1 B_1
   A_1 = cos(theta_1_one_r) * p_x_arm + sin(theta_1_one_r) * p_y_arm;
   B_1 = ( A_1^2+ p_z_arm^2 + a_2^2 -d_4^2 ) / (2*a_2);
   theta_2_one = atan2d(A_1, p_z_arm) - atan2d(B_1, +sqrt( A_1^2 + p_z_arm^2 -B_1^2) ); %  degree1
   theta_2_one_r = theta_2_one * 2 * pi /360; %  radian1
%% theta_3
     %  theta_2_one A_1 B_1
     coefficience_y_1 = A_1 - a_2 * cos( theta_2_one_r );
     coefficience_x_1 = p_z_arm + a_2 * sin( theta_2_one_r ) ;
     theta_3_one = ( atan2d ( coefficience_y_1 , coefficience_x_1 ) ) - theta_2_one; % degree1
     theta_3_one_r = theta_3_one * 2 * pi /360; %  radian1
 %% theta_4
     
      coefficience_y_1 = -sin(theta_1_one_r) * a_x + cos(theta_1_one_r) * a_y;
      coefficience_x_1 = cos( theta_2_one_r + theta_3_one_r ) * ( cos(theta_1_one_r) * a_x + sin(theta_1_one_r) * a_y )  ......
                         - sin( theta_2_one_r + theta_3_one_r ) * a_z;
      theta_4_one = atan2d ( coefficience_y_1 , coefficience_x_1 ); %degree1
      theta_4_one_r = theta_4_one * 2 * pi /360; %radian1
  %% theta_5
      
coefficience_x_1 = sin( theta_2_one_r + theta_3_one_r ) * ( cos(theta_1_one_r) * a_x + sin(theta_1_one_r) * a_y )......
                   + cos( theta_2_one_r + theta_3_one_r ) * a_z;
A_1 = cos(theta_1_one_r) * cos( theta_2_one_r + theta_3_one_r ) * a_x...
    + sin(theta_1_one_r) * cos( theta_2_one_r + theta_3_one_r ) * a_y...
    - sin( theta_2_one_r + theta_3_one_r ) * a_z;
B_1 = -sin(theta_1_one_r)*a_x + cos(theta_1_one_r) * a_y;
theta_5_one = atan2d( sqrt( A_1^2 + B_1^2 ) , coefficience_x_1 ); %degree1
theta_5_one_r = theta_5_one * 2 * pi /360; %radian1
%% theta_6

coefficient_y_1 = sin( theta_2_one_r + theta_3_one_r ) * ( cos(theta_1_one_r)*s_x + sin(theta_1_one_r)*s_y )...
                  + cos( theta_2_one_r + theta_3_one_r ) * s_z;
coefficient_x_1 = -( sin( theta_2_one_r + theta_3_one_r ) * ( cos(theta_1_one_r)*n_x + sin(theta_1_one_r)*n_y )...
                    + cos( theta_2_one_r + theta_3_one_r )*n_z );
theta_6_one = atan2d( coefficient_y_1 , coefficient_x_1  ); %degree1
theta_6_one_r = theta_6_one * 2 * pi /360; %radian1
%% Output
theta1=theta_1_one;
theta2=theta_2_one;
theta3=theta_3_one;
theta4=theta_4_one;
theta5=theta_5_one;
theta6=theta_6_one;