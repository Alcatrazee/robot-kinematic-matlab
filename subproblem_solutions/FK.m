function g_st_theta =  FK(angle_vector,length_of_links)
%% statement
% This function is to calculate forward kinematics of a six axis robot,and display it's skeleton
% The original pose of each joint is 0,and it's zero-angles position is
% like a L rotated 90 degrees in clock-wise.
% configuration of this six axis robot: rotating axis of joint 1 and joint 2 are
% perpandicular and intersect.Axis 3 is paralle to axis 2.Axis 4 and axis 6
% is initially paralle while axis 5 is perpendicular to these two axes.
%% input assert
% typical input 
%angle_vector = [-1.5708    0     0   -1.5708   -1.5708    3.1416];
%length_of_links = [0 2 3 0.5];
if sum(size(length_of_links)==[1 4])==2 || sum(size(length_of_links)==[4 1])==2
   disp('input length of links vector assert succeeded');
else
    error('input length of links vector assert failed.Input should be a vector of 6 columns or 6 rows');
end
if sum(size(angle_vector)==[1 6])==2 || sum(size(angle_vector)==[6 1])==2
    disp('input angle vector assert succeeded');
else
    error('input angle vector assert failed.Input should be a vector of 6 columns or 6 rows');
end
rad_vec = angle_vector;
%% parameters definition
% omega and q
rotation_axis_vecs = [0 0 1;
    1 0 0;
    1 0 0;
    0 1 0;
    1 0 0;
    0 1 0]';
q_vec = [0 0 length_of_links(1);
    0 0 length_of_links(1);
    0 0 length_of_links(1)+length_of_links(2);
    0 length_of_links(3) length_of_links(1)+length_of_links(2);
    0 length_of_links(3) length_of_links(1)+length_of_links(2);
    0 length_of_links(3) length_of_links(1)+length_of_links(2)]';
% -w x q
v_vec = zeros(3,1);
for i = 1:6
    v_vec = [v_vec, cross(-rotation_axis_vecs(:,i),q_vec(:,i))];
end
% [-w x q ; w]
twist_vec = [v_vec(:,2:7);rotation_axis_vecs];
%% define initial posture
g_st0 = [1 0 0 0;
    0 1 0 length_of_links(3)+length_of_links(4);
    0 0 1 length_of_links(1)+length_of_links(2);
    0 0 0 1];
%% calculate SE(3)
T1 = T_matrix(twist_vec(:,1),rad_vec(1));
T2 = T_matrix(twist_vec(:,2),rad_vec(2));
T3 = T_matrix(twist_vec(:,3),rad_vec(3));
T4 = T_matrix(twist_vec(:,4),rad_vec(4));
T5 = T_matrix(twist_vec(:,5),rad_vec(5));
T6 = T_matrix(twist_vec(:,6),rad_vec(6));

g_st_theta = T1*T2*T3*T4*T5*T6*g_st0;               % end-effector posture
%% create robot and visualize it with robotic toolbox
L1 = Link('alpha',0,'a',0,'d',length_of_links(1),'offset',pi/2,'modified');
L2 = Link('alpha',pi/2,'a',0,'d',0,'offset',pi/2,'modified');
L3 = Link('alpha',0,'a',length_of_links(2),'d',0,'modified');
L4 = Link('alpha',pi/2,'a',0,'d',length_of_links(3),'modified');
L5 = Link('alpha',-pi/2,'a',0,'d',0,'modified');
L6 = Link('alpha',pi/2,'a',0,'d',length_of_links(4),'modified');
robot = SerialLink([L1 L2 L3 L4 L5 L6]);
robot.plot(angle_vector);
%% draw framework of the robot
% Point1 = T1*[0 0 length_of_links(1) 1]';
% Point3 = T1*T2*T3*[0 0 length_of_links(1)+length_of_links(2) 1]';
% Point4 = T1*T2*T3*T4*[0 length_of_links(3) length_of_links(1)+length_of_links(2) 1]';
% Points_x = [0 Point1(1) Point3(1) Point4(1) g_st_theta(1,4)];
% Points_y = [0 Point1(2) Point3(2) Point4(2) g_st_theta(2,4)];
% Points_z = [0 Point1(3) Point3(3) Point4(3) g_st_theta(3,4)];
% plot3(Points_x,Points_y,Points_z,'marker','o')
% grid on
% axis equal
% xlabel('x')
% ylabel('y')
% max_axis_xy = length_of_links(2)+length_of_links(3);
% max_axis_z = length_of_links(1)+length_of_links(2)+length_of_links(3)+length_of_links(4);
% axis([-max_axis_xy,max_axis_xy,-max_axis_xy,max_axis_xy,0,max_axis_z]);
end