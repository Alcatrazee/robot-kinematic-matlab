function [theta0,theta1] = subproblem3_solution(r,q,p,w,g)
%% parameters : 
% r: a point in the rotation axis
% q: a point that never moves
% p: initial position of the point last parameter stated
% w: rotation axis
% g: final position of the point

% q = [0 3 2.5]';
% p = [0 3.5 2]';
% w3 = [1 0 0]';
% r = [0 3 2]';
%% calculation
u = p-r;
u_prime = u-w*w'*u;
v = q-r;
v_prime = v - w*w'*v;

theta0 = atan2(w'*cross(u_prime,v_prime),u_prime'*v_prime);
epsi_squre = norm(g - q)^2;

epsi_prime_squre = epsi_squre - norm(w'*(p-q))^2;
theta0 = theta0+acos((norm(u_prime)^2+norm(v_prime)^2 - epsi_prime_squre)/(2*norm(u_prime)*norm(v_prime)));
theta1 = theta0+acos((norm(u_prime)^2+norm(v_prime)^2 - epsi_prime_squre)/(2*norm(u_prime)*norm(v_prime)));

end