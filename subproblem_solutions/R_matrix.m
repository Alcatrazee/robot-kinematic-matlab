function result = R_matrix(axis,angle)
% this function is to calculate the rotation matrix rotate around only one 
% axis
% param1:axis is a char value, indicate which axis to rotate about
% param2:angle is the radian of the rotation
% with Rodriguez formula,it's easy to calculate the R matrix
% fact of this function: mapping from se(n) |-> SE(n), n = 3

%% fullfill

if(axis=='X' || axis=='x')
    result = expm(Matrix_hat([1 0 0])*angle);
elseif(axis=='Y'||axis=='y')
    result = expm(Matrix_hat([0 1 0])*angle);
elseif(axis=='Z'||axis=='z')
    result = expm(Matrix_hat([0 0 1])*angle);
end

end