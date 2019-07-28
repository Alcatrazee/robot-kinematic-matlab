function angle_vector = IK(g_st_theta,length_of_links)
%% statements
% This program is to calculate the inverse kinematic of a six axis robot
% arm with solution of Paden-Khan subproblem.
% Input parameters: 
% g_st_theta is the goal posture of the robot arm to solve.
% length_of_links is the vector of four links.
%% variables definition
% typical input of g_st_theta
% g_st_theta =[ 0.3846   -0.0000   -0.9231    2.0000
%              -0.0000   -1.0000    0.0000   -0.5000
%              -0.9231   -0.0000   -0.3846    3.0000
%                    0         0         0    1.0000];    % can be modified
% length_of_links = [1 2 3 0.5];
g_st0 = [1 0 0 0;
        0 1 0 length_of_links(3)+length_of_links(4);
        0 0 1 length_of_links(1)+length_of_links(2);
        0 0 0 1];                                       % initial position of end-effector
%% overall displacement: exp1*exp2*exp3*exp4*exp5*exp6 = g_st_theta*inv(g_st0)
g1 = g_st_theta/g_st0;       % damn bug occured,just because the inverse sign of '\' and '/'

%% step1.solve for theta3 with solution of subproblem 3
% We use wrist's position to reduce the affect of the motion created by joint 4 to
% joint 6,then we apply subproblem 3 to calculate the angle of theta 3
r3 = [0 0 length_of_links(1)+length_of_links(2)]';                      % point on the rotating axis
q3 = [0 0 length_of_links(1)]';                                         % point that never move
p3 = [0 length_of_links(3) length_of_links(1)+length_of_links(2)]';     % initial position of wrist
w3 = [1 0 0]';                                                          % axis 
g3 = g_st0(:,4);                                                        % final position of wrist
g3(2) = g3(2) - length_of_links(4);                                     
g3 = g1*g3;
[theta3(1),theta3(2)]= subproblem3_solution(r3,q3,p3,w3,g3(1:3));
twist3 = [cross(-w3,r3);w3];                        
exp3 = T_matrix(twist3,theta3(1));

%% step2.sovle for theta1 and theta2 with solution of subproblem 2
coincident_point = [0,0,length_of_links(1)]';                           % coincident point of two axis
w1 = [0 0 1]';
w2 = [1 0 0]';
original_position = exp3*[p3;1];                                        % original point of the movement
original_position = original_position(1:3);                             
final_position = g3(1:3);                                               % coordinate of a point after movement 

[theta1,theta2] = subproblem2_solution(coincident_point,w1,w2,original_position(1:3),final_position);

twist1 = [cross(-w1,[0 0 1]');w1];                                      % calculate twist of the product
exp1 = T_matrix(twist1,theta1(1));
twist2 = [cross(-w2,[1 0 0]');w2];
exp2 = T_matrix(twist2,theta2(1));

%% step3.solve for theta4 and theta5 with solution of subproblem 2
g2 = exp1*exp2*exp3\g_st_theta/g_st0;

final_p6 = g2*g_st0(:,4);
original_p6 = [0 length_of_links(3)+length_of_links(4) length_of_links(1)+length_of_links(2)]';
w4 = [0 1 0]';
w5 = [1 0 0]';
coincident_position5_4 = [0 length_of_links(3) length_of_links(1)+length_of_links(2)]';
[theta4,theta5] = subproblem2_solution(coincident_position5_4,w4,w5,original_p6(1:3),final_p6(1:3));

twist4 = [cross(-w4,[0 length_of_links(3) length_of_links(1)+length_of_links(2)]');w4];
exp4 = T_matrix(twist4,theta4(1));
twist5 = [cross(-w5,[0 length_of_links(3) length_of_links(1)+length_of_links(2)]');w5];
exp5 = T_matrix(twist5,theta5(1));
%% step4.solve for theta 6
exp6 = exp1*exp2*exp3*exp4*exp5\g_st_theta/g_st0;
% choose a random point that is not on the axis of joint 6
random_point = [0 0 1]';
q = exp6*[random_point ;1];
q = q(1:3);
r = [0 length_of_links(3) length_of_links(1)+length_of_links(2)]';
w = [0 1 0]';
p = random_point;
theta6 = subproblem1_solution(r,w,p,q);

angle_vector = [theta1(1) theta2(1) theta3(1) theta4(1) theta5(1) theta6];

gst_test = FK(angle_vector,length_of_links);
error = gst_test - g_st_theta
