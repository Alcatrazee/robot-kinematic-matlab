function SE_3 = T_matrix(twist,value)
% mapping from se(3) to SE(3)
% twist is a vector representing a motion parameter
% value is the radian that rotate or length that moves
% return exponential of a twist 

SE_3 = expm(Matrix_hat(twist)*value);

end