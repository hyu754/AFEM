function [ outputVec ] = find_index_structure( V,input_structure )
%this function finds the indices given a structure where .Position
%is the position of the points of interest. We will use the find_index_mesh
%function to find the individual position indicies.
%Input: -Mesh verticies
%       -structure.size >=1
%       
%Output: -vector with the indices

strucSize = length(input_structure);


%output vector with indices
outputVec=[];
for i = 1:strucSize
    dummy_var = find_index_mesh(V,input_structure(i).Position);
    outputVec=[outputVec dummy_var];
end



end

