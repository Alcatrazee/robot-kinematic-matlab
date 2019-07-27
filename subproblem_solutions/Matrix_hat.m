function result = Matrix_hat(vector)
% this function is to calculate the hat matrix of a vector
% input vector of 3 or 2 dimension

%% instance of function
if sum(size(vector)==[1 3])==2||sum(size(vector)==[3 1])==2
    result = [0 -vector(3) vector(2);
              vector(3) 0 -vector(1);
              -vector(2) vector(1) 0];
elseif sum(size(vector)==[1 6])==2||sum(size(vector)==[6 1])==2
    temp =  [0 -vector(6) vector(5);
              vector(6) 0 -vector(4);
              -vector(5) vector(4) 0];
    mat_bottom = zeros(1,4);
    temp_concate = [temp [vector(1) vector(2) vector(3)]'];
    result = [temp_concate;mat_bottom];
else
    error('input vector assert failed.Please check the input vector for Matrix_hat function');
end
end