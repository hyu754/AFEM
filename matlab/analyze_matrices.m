fid = fopen('../build/LHS.txt','r');
a = importdata('../build/LHS.txt');
[m,n] = size(a);
%if non-symmetric, display which ones
if(issymmetric(a)==0)
    for i = 1:m
       for j =1:n
          if(a(i,j) ~= a(j,i))
             disp(['Non symmetric element: \n']);
             disp(['i = ',num2str(i),', j = ', num2str(j),'\n']);
             disp([num2str(a(i,j)),' ~= ' , num2str(a(j,i)), '\n']);
             
          end
       end
    end
else
    disp('This matrix is symmetrical');
end
