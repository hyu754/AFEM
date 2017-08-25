clear
clc
close all


%% THIS SCRIPT WILL READ THE OUTPUT FROM AFEM


%% Get information stereo vision
% AFEM_OUTPUT - folder will contain files that store positions of tumours
% time
% x_1 y_1 z_2
% ..etc
disp('Getting information stereo ...');
dir_result = dir('AFEM_OUTPUT');

numFiles = length(dir_result);

%number of nodes in each file
fileNodeNumberStereo =0;

%This will store the result from reading all of the files
%This is for stereo vision
pos_storage_stereo={};
time_vector_stereo=[];

%Read in files for stereo
for i = 1: numFiles
    if (~isempty(strfind(dir_result(i).name,'.txt')))
        if (~isempty(strfind(dir_result(i).name,'stereo')))
            file_name =['AFEM_OUTPUT/',char(dir_result(i).name)];

            datafile = importdata(file_name);

            time = datafile(1);
            pos = datafile(2:end);

            pos = reshape(pos,3,[])';
            fileNodeNumberStereo = size(pos,1);

            %put above information into global vector
            time_vector_stereo = [time_vector_stereo time];
            pos_storage_stereo{length(pos_storage_stereo)+1} = pos;
        end
        
    end
end

%% Get information finite element solver
% AFEM_OUTPUT - folder will contain files that store positions of tumours
% time
% x_1 y_1 z_2
% ..etc
disp('Getting information fem ...');
dir_result = dir('AFEM_FEM_OUT');

numFiles = length(dir_result);

%number of nodes in each file
fileNodeNumberFEM =0;

%This will store the result from reading all of the files
pos_storage_fem={};

%Read in files for stereo
for i = 1: numFiles
    disp(['iteration : ', num2str(i)]);
    if (~isempty(strfind(dir_result(i).name,'.txt')))
        if (~isempty(strfind(dir_result(i).name,'fem')))
            file_name =['AFEM_FEM_OUT/',char(dir_result(i).name)];
           % disp(file_name);
            datafile = importdata(file_name);

        
            pos = datafile(1:end);

            pos = reshape(pos,[],3);
            
            fileNodeNumberFEM = size(pos,1);

            %put above information into global vector
       
            pos_storage_fem{length(pos_storage_fem)+1} = pos;
        end
        
    end
end





%% Plot mesh stereo

%original position
%find the avrage initial position
mean_orig = zeros(size(pos_storage_stereo{1},1),size(pos_storage_stereo{1},2))
for i = 1:5
  mean_orig = mean_orig + pos_storage_stereo{i}
end
mean_orig = mean_orig/5;
orig_pos_stereo = mean_orig;

%number of data points
numDataPointsStereo = length(pos_storage_stereo);

%Diff list
diff_list ={};

counter_list =1;
figure
title('Stereo traject for tumours')
xlabel('Time ')
ylabel('Norm of displacement (m)');

hold on
for i = 2:numDataPointsStereo
   diff_mat  =orig_pos_stereo - pos_storage_stereo{i};
    scatter3(pos_storage_stereo{i}(1,1),pos_storage_stereo{i}(1,2),pos_storage_stereo{i}(1,3))
   
   diff_list{counter_list} = diff_mat;
   counter_list = counter_list + 1;
end

figure
title('Stereo plots for tumours')
xlabel('Time ')
ylabel('Norm of displacement (m)');

hold on
for i = 1:numDataPointsStereo
   temp_vector=[]
   for j = 1:length(diff_list)
       temp_vector = [temp_vector ;norm(diff_list{j}(i,:))];
   end    
   plot(smooth(temp_vector));
    pause(0.2);
    if i == 5
        break
    end
end

legend('0','1','2','3','4')

figure
title('stereo displacement vector for tumours')
axis equal
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
hold on
for i = 1:numDataPointsStereo
   temp_vector=[]
   max_value = -1;
   max_id =0;
   for j = 1:length(diff_list)
       temp_vector = [temp_vector ;norm(diff_list{j}(i,:))];
       if (norm(diff_list{j}(i,:))>max_value)
          max_value =  norm(diff_list{j}(i,:));
          max_id = j;
       end
   end    
   displacement_vector = -diff_list{max_id}(i,:);
   plot3([0 displacement_vector(1)],[0 displacement_vector(2)],[0 displacement_vector(3)]);
   
    pause(0.2);
    if i == 5
        break
    end
end

legend('0','1','2','3','4')




%% Plot mesh FEM

%original position
orig_pos_fem = pos_storage_fem{1};

%number of data points
numDataPointsFEM = length(pos_storage_fem);

%Diff list
diff_list ={};

counter_list =1;
figure
title('FEM trajectory for tumours')
xlabel('x');
ylabel('y')
zlabel('z')
hold on
for i = 1:numDataPointsFEM
   diff_mat  =orig_pos_fem - pos_storage_fem{i};
   scatter3(pos_storage_fem{i}(3,1),pos_storage_fem{i}(3,2),pos_storage_fem{i}(3,3))
   axis equal
   %scatter3(pos_storage_fem{i}(2,1),pos_storage_fem{i}(2,2),pos_storage_fem{i}(2,3))
   
   diff_list{counter_list} = diff_mat;
   counter_list = counter_list + 1;
end

figure
title('FEM plots for tumours')
xlabel('Time ')
ylabel('Norm of displacement (m)');



hold on
for i = 1:numDataPointsFEM
   temp_vector=[]
   for j = 1:length(diff_list)
       temp_vector = [temp_vector ;norm(diff_list{j}(i,:))];
   end    
   plot(smooth(temp_vector));
    pause(0.2);
    if i == 5
        break
    end
end

legend('0','1','2','3','4')

%Displacement vector
figure
title('FEM displacement vector for tumours')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')

hold on
for i = 1:numDataPointsFEM
   temp_vector=[]
   max_value = -1;
%    max_id =0;
%    for j = 1:length(diff_list)
%        temp_vector = [temp_vector ;norm(diff_list{j}(i,:))];
%        if (norm(diff_list{j}(i,:))>max_value)
%           max_value =  norm(diff_list{j}(i,:));
%           max_id = j;
%        end
%    end    

%Steady displacement is at the end
   displacement_vector = -diff_list{end}(i,:);
   plot3([0 displacement_vector(1)],[0 displacement_vector(2)],[0 displacement_vector(3)]);
   
    pause(0.2);
    if i == 5
        break
    end
end


legend('0','1','2','3','4')





