i=7;
    
[T,V] = regular_tetrahedral_mesh(i,i,2);

% d = linspace(0,1,10);
% d2 = linspace(0,1,15);
% [x,y,z] = meshgrid(d,d,d2);  % A cube
% x = [x(:);0];
% y = [y(:);0];
% z = [z(:);0];
% % [x,y,z] are corners of a cube plus the center.
% X = [x(:) y(:) z(:)];
% Tes = delaunayn(X)
% T = Tes
% V=X


V(:,1)=V(:,1)
if(0)
    
    T=newelem(:,[1:4]);
    V = newnode
    V=V/1000;
else
    V=V/10;
end

%if we are using matlab mesh generator
if(0)
    model = createpde(1);
importGeometry(model,'cube.stl');
mesh_generated= generateMesh(model,'Hmax',0.07)
mesh_element = mesh_generated.Elements'
mesh_nodes = mesh_generated.Nodes'
% [n,m] = size(mesh_element);
% figure
% hold on
% 
% for i = 1:n
%  
%          scatter3( mesh_nodes(mesh_element(i,1:4),1),mesh_nodes(mesh_element(i,1:4),2),mesh_nodes(mesh_element(i,1:4),3));
% 
% end
% 
% pdeplot3D(model)


V = mesh_nodes;
T = mesh_element(:,1:4);
end

 V=V*1000;
 V(:,1) = 0.67*V(:,1);
 V(:,2) = 0.90*V(:,2);
V(:,1)=V(:,1)%-mean(V(:,1));
V(:,1)=V(:,1)
V(:,1)=V(:,1);
V(:,2) = V(:,2);

V(:,3) =V(:,3)*0.08;


% V(:,1)=V(:,1);
% V(:,2)=V(:,2)/2;
% V(:,3)=V(:,3)/2%+\\1.0;
% 
% V(:,3)= V(:,3)-(max(V(:,3))-min(V(:,3)))/2;
% V(:,2)= V(:,2)-(max(V(:,2))-min(V(:,2)))/2;
%V(:,1)= V(:,1)-(max(V(:,1))-min(V(:,1)))/2;


if(0)
    
   dnode = max(node)-min(node); 
    node(:,1) = node(:,1) - min(node(:,1));
    node(:,2) = node(:,2) - min(node(:,2));
    node(:,3) = node(:,3) - min(node(:,3));
    
    node(:,1)= node(:,1)/dnode(1);
    node(:,2)= node(:,2)/dnode(2);
    node(:,3)= node(:,3)/dnode(3);
      
    V = node/10;
      
      V(:,1)=V(:,1)*2.5;
      T = elem(:,[1:4])
end


dv = max(V)-min(V)
Volume = dv(1)*dv(2)*dv(3)
tetramesh(T,V); 
xlabel('x');
ylabel('y');
zlabel('z');



%Test ray surf
x=[1];
y=[1];
p0=[0.0;0.46;0.2]';
hold on
scatter3(p0(1),p0(2),p0(3));

v0=[0,0,-1];
%generate face
[numelem,~] = size(T);
faces_unfiltered=[];
for i = 1:numelem
    elem_selected = T(i,:);
    
   faces_unfiltered = [faces_unfiltered; elem_selected([1,2,3]) i ; elem_selected([1,4,2]) i;elem_selected([2,4,3]) i;elem_selected([1,3,4]) i];
  
end
[num_unfiltered_faces,~] = size(faces_unfiltered);
faces=[];
%   for j = 1:num_unfiltered_faces
%       exists = false;
%            for n = 1:size(faces,1)
%               if sum(sort(faces_unfiltered(j,:)) == sort(faces(n,:))) == 4
%                 exists=true;
%               end
%            end
%            if exists == false
%                
%                 faces = [faces;  faces_unfiltered(j,:)];
%                 
%            end
%     end
face = faces_unfiltered;

if(0)
    
[t,u,v,idx]=raytrace(p0,v0,V,face(:,1:3))
p0 = [21 21 200]
[t,u,v,idx,xnode]=raysurf(p0,v0,V,face(:,1:3))
%
%give id of minimimum distance

[~,id] = min(t);

face_selected = face(idx,:);


scatter3(V(face_selected(1:3),1),V(face_selected(1:3),2),V(face_selected(1:3),3));
barycentric = [u,v,1-u-v];

coordinate_mat = V(face_selected(1:3),:)'

actual_barycentric= inv(coordinate_mat)*xnode';



actual_coord = coordinate_mat * actual_barycentric;
scatter3(actual_coord(1),actual_coord(2),actual_coord(3))

%now interms of 4 point tetrahedral points

face_selected = face(idx,:);
element_id = face_selected(4);
element_indicies = T(element_id,:);

coordinate_mat = V(element_indicies,:)';
coordinate_mat = [coordinate_mat;[1 1 1 1]]
barycentric4nodes = inv(coordinate_mat)* [xnode 1]';
n1 = barycentric4nodes(1);
n2 = barycentric4nodes(2);
n3 = barycentric4nodes(3);

n1 = 0.1;
n2 = 0.2;
n3 =0.3;
n4 = 1 - n1 - n2- n3;
N = [[n1 0  0  n2 0  0  n3 0  0  n4 0  0 ];
     [0  n1 0  0  n2 0  0  n3 0  0  n4 0 ]; 
     [0  0  n1 0  0  n2 0  0  n3 0  0  n4]];
 
 pseudo_N_inv =inv(transpose(N)*N) *transpose(N)

 N_square = eye(12);
 N_square(1:3,:) = N;
 
 N_square_inv = inv(N_square)
 
N_transpose = N'/(n1*n1 + n2*n2 + n3*n3 + n4*n4);


x_vec=reshape(coordinate_mat(1:3,:),1,12)
y_vec = N_square*x_vec'


x_vec_res = N_square_inv * y_vec;
end
%file_path='D:\GitHub\opencloth_vision\Geometry\';
file_path='C:\Users\hyu754\Dropbox\cuda_solver_independent_magma\cuda_solver_independent\';
file_path='C:\Users\hyu754\Documents\Visual Studio 2013\Projects\PCL2 - Copy\PCL2\';
file_path = 'C:\Users\hyu754\Downloads\opencloth_vision\opencloth_vision\build\'
file_path = 'C:\Users\hyu754\Downloads\opencloth_vision\opencloth_vision\Geometry\'
file_path = 'D:\GitHub\AFEM\Geometry\';
file_path = 'D:\GitHub\AFEM\Geometry\';


file_path_stl_out = 'D:\GitHub\AFEM\build\geometry.stl';




A = V(V(:,3)==0,:,:);
 compare = V(:,2)<0.0170205
 compare2 = V(:,1)<20
% compare3 = V(:,2)>0.29
% compare4 = V(:,1)>0.29
% compare5= V(:,3) < 0.01
% compare = compare|compare2
% compare = compare|compare3
% compare = compare|compare4
 compare = compare % &compare2
%compare = V(:,3)<0.0012
[m,n] = size(V);
stationary =[];
counter =1;
for i = 1:m
    if compare(i)
      stationary(counter)=i-1;
        counter= counter+1;
    end
    
end


a=sprintf('%d,',stationary);

a = strcat('{',a,'}')


p=V;
t=T;
[num_points,n]= size(p);
vector = 0:num_points*3-1;
%dummy = vec2mat(vector,3);

fileID = fopen([file_path,'FEM_Nodes.txt'],'w');
fprintf(fileID,'%d\n',num_points);
fprintf(fileID,'%f %f %f\n',p'/1000);
fclose(fileID);
[numelem,~]=size(t);
t=t-1;
fileID = fopen([file_path,'FEM_Elem.txt'],'w');
fprintf(fileID,'%d  4\n',numelem);



factor_vector=divisors(numelem)
factor_vector2=numelem./factor_vector
fprintf(fileID,'%d %d %d %d\n',t');
fclose(fileID);


fileID = fopen([file_path,'FEM_Stationary.txt'],'w');
fprintf(fileID,'%d \n',length(stationary));
fprintf(fileID,'%d \n',stationary);
fclose(fileID);

% fileID = fopen(['C:/Users/hyu754/Dropbox/cuda_solver_independent_magma/cuda_solver_independent/FEM_displacement', num2str(i),'.txt'],'w');
% fprintf(fileID,'%d %d %d\n',dummy');
% fclose(fileID);
% t=t+1;

%
%u_file = load('C:/Users/hyu754/Dropbox/cuda_solver_independent_magma/cuda_solver_independent/FEM_position_result.txt');
%[m,n]=size(u_file);
%scatter(p(:,1),p(:,2));
%u=[];
%numberpoints = num_points;
%number = m/numberpoints;
%
%
%for j = 1:number
%x = u_file((1:numberpoints)+(j-1)*numberpoints,1);
%y= u_file((1:numberpoints)+(j-1)*numberpoints,2);
%p=[x y];
%simpplot(p,t)
%
%%hold on
%%plot([x(1) x(3) x(2) x(4) x(5) x(3) x(2) x(1)],[y(1) y(3) y(2) y(4) y(5) y(3) y(2) y(1)]);
%axis([-2 2 -2 2])
%pause(1/30)
%hold off
%end
%
%%for i = 1:length(u_file)/2
%%  u=[u;u_file(1+(i-1)*2) u_file(2+(i-1)*2)];
%%  
%%end
%
