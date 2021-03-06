% This function will read in CMGUI exnode, and output the node's x,y,z
% position
% The variable startline will determine where the code actually starts
function [x,y,z,nodeNumber,no_node] = readnode_CMGUI(file,startline)


    %Read in the exnode file
    fid = fopen(file);
    %Find the line where the node coordinates begin
    nodeBool = false;

    % while (~nodeBool)
    %     tline = fgets(fid);
    %     for i = 1:length(tline)-4
    %         if (strcmp(tline([i i+1 i+2 i+3]), 'Node'))
    %             nodeBool = true;
    %         end
    %     end
    % end

    for i = 1:startline
        tline = fgets(fid);
    end
    i = 1;
    nodeNumber(1) = 0;
    while (1)
        k= fscanf(fid, [' Node: %d']);
        if (isempty(k))
            break;
        else
            nodeNumber(i) =k;
        end
        x(i) = fscanf(fid, '%f',[1]);%d1(i,[1 2 3]) = fscanf(fid, '%f',[3]);fgets(fid);
        y(i) = fscanf(fid, '%f',[1]);%d2(i,[1 2 3]) = fscanf(fid, '%f',[3]);fgets(fid);
        z(i) = fscanf(fid, '%f',[1]);%d3(i,[1 2 3]) = fscanf(fid, '%f',[3]);fgets(fid);
        i = i+1;
    end

    no_node = length(nodeNumber);
    initialPosition = [ x',y',z'];
    fclose(fid);
end
