function [theta1,theta2] = subproblem2_solution(coincident_point,w1,w2,original_position,final_position)
%% expliation
%
%% known variables
% w1 = [0 0 1]';
% w2 = [1 0 0]';
% original_position = [0 3 2]';
% final_position = [2 0 3]';
% coincident_point = [0 0 0]';

u = original_position - coincident_point;
v = final_position - coincident_point;
alpha = ((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
beta =  ((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
gamma_squre = (norm(u)^2-alpha^2-beta^2-2*alpha*beta*w1'*w2)/(norm(cross(w1,w2))^2);
gamma = sqrt(gamma_squre);
z = alpha*w1+beta*w2+gamma*(cross(w1,w2));

c = z+coincident_point;

theta2(1) = subproblem1_solution(coincident_point,w2,original_position,c);
theta1(1) = subproblem1_solution(coincident_point,w1,c,final_position);

gamma2 = -gamma;
z2 = alpha*w1+beta*w2+gamma2*(cross(w1,w2));
c2 = z2+coincident_point;

theta2(2) = subproblem1_solution(coincident_point,w2,u,c2);
theta1(2) = subproblem1_solution(coincident_point,w1,c2,v);



