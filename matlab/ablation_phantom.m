
clear all
close all

%% This script will generate the mesh of the abdominal phantom
%add directory to iso2mesh and samples
addpath('D:\Downloads\iso2mesh\')
addpath('D:\Downloads\iso2mesh\sample\')
addpath('D:\abdominal_image\DICOM\20170626\15070000\parta\')


% [f,v] = stlread('D:\abdominal_image\DICOM\20170626\15070000\parta\outersurface.stl');

%Generate geometry using MATLAB's internal PDE generator
model = createpde(1);
importGeometry(model,'D:\abdominal_image\DICOM\20170626\15070000\parta\outersurface.stl');
mesh_generated =generateMesh(model,'Hmax',20,'GeometricOrder','linear');
%Plot the model
%pdeplot3D(model,'FaceAlpha',0.5);

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

%% Refine again to make force node precise

%start position of node
force_start = [146.384 228.63 65.3606];
%direction
force_dir = [0 -1 0];
[t,u,v,idx,xnode]=raysurf(force_start,force_dir,newnode,newface);

%find closest mesh node and move it to position of intersection
min_distance = 1000000;
min_id = 0;
for i = 1:size(newnode,1)
   diff_vec = newnode(i,:)- xnode;
   if(norm(diff_vec)<min_distance)
      min_distance = norm(diff_vec); 
      min_id = i;
   end
end

newnode(min_id,:) = xnode;


%% Find the node number of each of the above inserted nodes

%Number of nodes
[numNodes,~] = size(newnode);
%Number of elements
[numElem,~ ] = size(newelem);
%Number of insertion points
numInsertion = length(x_concat);
for i=1:numNodes
    for j = 1: numInsertion
        
        if (norm(newnode(i,:)- [x_concat(j) y_concat(j) z_concat(j)])<0.0001)
            %disp('equal')
        end
    end
end


%% Find the plane that will act as direchlet BC
%make a plane based on the points picked
draw_plane = false; 
figure
hold on
x_=0:1:300;
[X,Y] = meshgrid(x_);
[a1,b1,c1,d1] = find_equation_plane([0 0 8],[0 1 8],[1 0 8]);
Z=(d1- a1 * X - b1 * Y)/c1;
if(draw_plane == true)
    surf(X,Y,Z)    
end

%shading flat
xlabel('x'); ylabel('y'); zlabel('z')


[a2,b2,c2,d2] = find_equation_plane([0 0 110],[0 1 110],[1 0 110]);
Z=(d2- a2 * X - b2 * Y)/c2;
if(draw_plane == true)
    surf(X,Y,Z)    
end
%shading flat
xlabel('x'); ylabel('y'); zlabel('z')
compare = zeros(1,numNodes);
compare_tumour = zeros(1,numNodes);
stationary_nodes=[]

%additional boundary values
additional_boundary_values=[]
additional_boundary_values = [x_ribs',y_ribs',z_ribs'];
additional_boundary_values  = [additional_boundary_values; x_vertebrae', y_vertebrae', z_vertebrae'];

tumour_position = [x_tumour', y_tumour',z_tumour'];

%node ids for the points that are on the side of the phantom
sideConstraintId =[];
for l = 1:numNodes
    %check if it is near anyplanes
    diff1 = dot(newnode(l,:),[a1 b1 c1])-d1;
    diff2 = dot(newnode(l,:),[a2 b2 c2])-d2;
    if ((norm(diff1)<4)||(norm(diff2)<4))
        compare(l)=1;
        sideConstraintId = [ sideConstraintId l];
        stationary_nodes = [stationary_nodes;newnode(l,:)];
    else
        compare(l)=0;
        
    end
    
    %for the base
    if(newnode(l,2)<20)
        
       % compare(l) = 1;
         % stationary_nodes = [stationary_nodes;newnode(l,:)];
        
    end
    
    for j = 1:size(additional_boundary_values,1)
       if norm(newnode(l,:)- additional_boundary_values(j,:)) <0.001
          compare(l)=1; 
          stationary_nodes = [stationary_nodes;newnode(l,:)];
          %disp('equal');
       end
    end
    
    for j = 1:size(tumour_position,1)
       if norm(newnode(l,:)- tumour_position(j,:)) <0.001
          compare_tumour(l)=1; 
          
       end
    end
    
end



%stationary id
stationary =[];
%tumour id
tumour_id = [];
counter =1;
for i = 1:numNodes
    if compare(i)
        stationary(counter)=i-1;
        counter= counter+1;
    end
    
    if compare_tumour(i)
        tumour_id = [tumour_id (i-1)];
        
    end
    
end



%% Plot the result
% plot the nodes 
%  scatter3(newnode(:,1),newnode(:,2),newnode(:,3),'b');
%     


% plot the stationary nodes on the sides
scatter3(stationary_nodes(:,1),stationary_nodes(:,2),stationary_nodes(:,3),'r');

% plot tumour position
scatter3(x_tumour,y_tumour,z_tumour,150,'filled')

% % plot vertebrae position
% scatter3(x_vertebrae,y_vertebrae,z_vertebrae,200,'filled');
% 
% % plot ribs position
% scatter3(x_ribs,y_ribs,z_ribs, 300, 'filled');

%Plot the new mesh
plotmesh(newnode,newelem)

%plot force node
scatter3(force_start(1),force_start(2),force_start(3),20)

%set alpha
alpha 0.5


%% Write to FEM solver

file_path = 'D:\GitHub\AFEM\Geometry\';

fileID = fopen([file_path,'FEM_Nodes.txt'],'w');
fprintf(fileID,'%d\n',numNodes);
fprintf(fileID,'%f %f %f\n',newnode'/1000);
fclose(fileID)

fileID = fopen([file_path,'FEM_Elem.txt'],'w');
fprintf(fileID,'%d  4\n',numElem);
fprintf(fileID,'%d %d %d %d\n',newelem'-1);
fclose(fileID);

fileID = fopen([file_path,'FEM_Stationary.txt'],'w');
fprintf(fileID,'%d \n',length(stationary));
fprintf(fileID,'%d \n',stationary);
fclose(fileID);


fileID = fopen([file_path,'FEM_Tumour.txt'],'w');
fprintf(fileID,'%d \n',length(tumour_id));
fprintf(fileID,'%d \n',tumour_id);
fclose(fileID);

fileID = fopen([file_path,'FEM_SideConstraints.txt'],'w');
fprintf(fileID,'%d \n',length(sideConstraintId));
fprintf(fileID,'%d \n',sideConstraintId-1);
fclose(fileID);





