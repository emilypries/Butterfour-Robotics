%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics Fall 2014
%
% Homework 4
%
% Team Number:  4
% Team Leader:  Luis Tolosa (let2120)
% Team Members: Emily Pries (ep2642)
%               Ernesto Sandoval Castillo (es3187) 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
% main function - creates graph representation of environment
% file - a file formatted as described in the assignment instructions
function path_planner(obj_file, pts_file)
    supermatrix = [];
    supernums   = [];

    points = fopen(pts_file);
    line = fgetl(points);
    sp = textscan(line,'%s');
    sp = sp{1};
    start = [str2double(sp(1)) str2double(sp(2))];
    line = fgetl(points);
    sp = textscan(line,'%s');
    sp = sp{1};
    goal = [str2double(sp(1)) str2double(sp(2))];
    fclose(points);
    
    objects = fopen(obj_file);
    nobj  = str2double(fgetl(objects)) - 1
    nwall = str2double(fgetl(objects))
    vwall = [];
    for i = 1:nwall
        line = fgetl(objects);
        sp = textscan(line,'%s');
        sp = sp{1};
        vwall(i,:) = [str2double(sp(1)) str2double(sp(2))];
    end 
    for j=1:nobj
        supernums(j) = str2double(fgetl(objects));
        for k=1:supernums(j)
            line = fgetl(objects);
            sp = textscan(line,'%s');
            sp = sp{1};
            supermatrix(sum(supernums)-supernums(j)+k,:) = ...
                                    [str2double(sp(1)) str2double(sp(2))];
        end
    end    
    fclose(objects);
    
    fh = figure('Name', 'Map');    
    hold on;
    fill(vwall(:,1),vwall(:,2),'w');
    rows = 0;
    for n=1:nobj
        fill(supermatrix(rows+1:rows+supernums(n),1),...
             supermatrix(rows+1:rows+supernums(n),2),...
             'r')
         rows = rows + supernums(n);
    end
    plot(start(1), start(2), '+', 'markersize', 13);
    plot(goal(1), goal(2), '+', 'markersize', 13);
    camroll(90);
end