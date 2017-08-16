function [ output_args ] = find_index_mesh( v, x )
%find_index_mesh this function will find the index given the output
%vertices and the point of interest x.
%output will be inf if nothing is found, and will output where 0 is the
%starting index.
output_args = inf;
[m,~]= size(v);
for i=1:m
   if(norm(v(i,:)-x)<0.01)
      output_args = i-1; %we minus one because output will be for C indices 
   end
    
end


end

