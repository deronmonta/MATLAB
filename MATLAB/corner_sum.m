function s = corner_sum(A)
%find the sum of the four corners of a matrix 
[M,N] = size(A);
s= A(1,1) + A(1,M) + A(N,1) + A(M,N);
end
