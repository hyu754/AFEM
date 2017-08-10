%% This script will generate the mesh of the abdominal phantom 
%add directory to iso2mesh and samples
addpath('D:\Downloads\iso2mesh\')
addpath('D:\Downloads\iso2mesh\sample\')
addpath('D:\abdominal_image\DICOM\20170626\15070000\parta\')


% [f,v] = stlread('D:\abdominal_image\DICOM\20170626\15070000\parta\outersurface.stl');

%Generate geometry using MATLAB's internal PDE generator
model = createpde(1);
importGeometry(model,'D:\abdominal_image\DICOM\20170626\15070000\parta\outersurface.stl');
mesh_generated =generateMesh(model,'Hmax',20);
%Plot the model
pdeplot3D(model,'FaceAlpha',0.5);

%Assign class variables 
mesh_element = mesh_generated.Elements';
mesh_nodes = mesh_generated.Nodes';




%% Add internal structure information


% Tumour positions
[x_tumour,y_tumour,z_tumour,nodeNumber] = readnode_CMGUI('tumour.exnode',7);

% Vertebrae positions
[x_vertebrae,y_vertebrae,z_vertebrae] = readnode_CMGUI('vertebrae.exnode',7);

% Ribs positions
[x_ribs,y_ribs,z_ribs] = readnode_CMGUI('ribs.exnode',7);

%Add new nodes with meshrefine
%These will be the concat of the above position vecs
x_concat = [ x_tumour x_vertebrae x_ribs];
y_concat = [ y_tumour y_vertebrae y_ribs];
z_concat = [ z_tumour z_vertebrae z_ribs];




[newnode,newelem,newface]=meshrefine(mesh_nodes,mesh_element(:,1:4),[],[x_concat',y_concat',z_concat']);

%Number of nodes
[numNodes,~] = size(newnode);
%Number of insertion points
numInsertion = length(x_concat);
for i=1:numNodes
   for j = 1: numInsertion
       
       if (norm(newnode(i,:)- [x_concat(j) y_concat(j) z_concat(j)])<0.0001)
          disp('equal') 
       end
   end
end


%% Plot the result

figure
hold on
% plot tumour position
scatter3(x_tumour,y_tumour,z_tumour,150,'filled')

% plot vertebrae position
scatter3(x_vertebrae,y_vertebrae,z_vertebrae,200,'filled');

% plot ribs position
scatter3(x_ribs,y_ribs,z_ribs, 300, 'filled');

%Plot the new mesh
plotmesh(newnode,newelem)
%set alpha
alpha 0.7





