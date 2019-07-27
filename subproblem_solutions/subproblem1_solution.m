function theta = subproblem1_solution(r,w,p,q)
%% explaination
% Solution to Paden-Khan subproblem 1
% Subproblem 1 is to solve for the angle that rotates around an axis  
% Input parameter:
% r is the point on the axis
% w is the axis that rotate around
% p is original position of a point 
% q is the position of the point after rotation

%% known variables
% syms xp yp zp xq yq zq
% p = [xp yp zp]';
% q = [xq yq zq]';
% p = [0 1 1]';
% q = [-sqrt(2) sqrt(2)  1]';
% r = [0,0,0]';       % a point on rotating axis
%% middle variables 
u = p - r;
v = q - r;
u_prime = u - w*w'*u;
v_prime = v - w*w'*v;
%% result
theta = atan2(w'*(cross(u_prime,v_prime)),u_prime'*v_prime);