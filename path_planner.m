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
% obj_file - a file containing a descirption of the environment obstacles
% pts_file - a file containing the start and goal points for path-plannning
function path_planner(obj_file, pts_file)
    supermatrix = [];   % holds object vertices in order read from file
    supernums   = [];   % holds number of vertices per object 
    ROOMBA_DIAM = 0.35; % diamater of roomba

    % get start and goal points from pts_file
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
    
    % get obstacle vertices for each object from obj_file
    objects = fopen(obj_file);
    nobj  = str2double(fgetl(objects)) - 1;
    nwall = str2double(fgetl(objects));
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
    
    grownmatrix = [];    % holds grown object vertices
    
    % treating the roomba as a square, the would-be reflection is the same
    % we grow the objects with this 'reflection' of robot
    rows = 0;
    for i = 1:nobj
        obj_v = [supermatrix(rows+1:rows+supernums(i),1),...
                 supermatrix(rows+1:rows+supernums(i),2)];
        center = [mean(obj_v(:, 1)), mean(obj_v(:, 2))];
        for j = 1:supernums(i)
            xy = obj_v(j, :);
            xy = xy - center;
            x = xy(1); y = xy(2);
            if(x < 0)
                x = x - ROOMBA_DIAM;
                if(y < 0)
                    y = y - ROOMBA_DIAM;
                elseif(y > 0)
                    y = y + ROOMBA_DIAM;
                end 
            elseif(x > 0)
                x = x + ROOMBA_DIAM;
                if(y < 0)
                    y = y - ROOMBA_DIAM;
                elseif(y > 0)
                    y = y + ROOMBA_DIAM;
                end 
            end    
            obj_v(j, :) = [x+center(1), y+center(2)];
        end
        obj_v
        % convex hull of grown object
        grownmatrix = [grownmatrix; convex_hull(obj_v)];
        rows = rows + supernums(i);
       
    end
    %Run dijkstra
    [path, path_dist, dist_array] = dijkstra([start; goal; grownmatrix]);
    pa = [start; goal; grownmatrix];
    [palength, pawidth] = size(pa);
    disp(dist_array)
    
    % create visibility graph
    fh = figure('Name', 'Map');    
    hold on;
    fill(vwall(:,1),vwall(:,2),'w');
    rows = 0;
    for n=1:nobj
        fill(grownmatrix(rows+1:rows+supernums(n),1),...
             grownmatrix(rows+1:rows+supernums(n),2),...
             'y')
        fill(supermatrix(rows+1:rows+supernums(n),1),...
             supermatrix(rows+1:rows+supernums(n),2),...
             'r')
         rows = rows + supernums(n);
    end
    % Draw Visibility Graph
    for i = 1:palength
        for j = i:palength
            if (dist_array(i,j) < inf)
                %TODO: draw line from pa(i,1) pa(i,2) to pa(j,1) pa (j,2)
                disp('drew edge')
                plot([pa(i,1), pa(j,1)], [pa(i,2), pa(j,2)], 'k');
            end
        end
    end
    % Draw the Path
    [pathrow, pathlength] = size(path);
    for i = 1:(pathlength-1)
        disp('drew path edge')
        plot([pa(path(i),1), pa(path(i+1),1)],[pa(path(i),2), pa(path(i+1),2)], 'm');
    end
    plot(start(1), start(2), '+', 'markersize', 13);
    plot(goal(1), goal(2), '+', 'markersize', 13);
    camroll(90);
    
              
end

% grows an obstacle given the object's vertex coordinates
function grown = convex_hull(coords)
    % find the rightmost, lowest point of object, p_zero
    p_zero = coords(1,:);
    for i=2:size(coords,1)
        new_p = coords(i,:);
        if new_p(1) >= p_zero(1) && new_p(2) <= p_zero(2)
            p_zero = new_p;
        end
    end
    p_zero
    [boo,idx] = ismember(p_zero, coords, 'rows');
    rest_ps = [coords(1:idx-1, :);...
               coords(idx+1:size(coords,1), :)];
    rest_ps = [rest_ps zeros(size(rest_ps,1),1)];
    baseline = [1 0];
    for j=1:size(rest_ps,1)
        vect_p = rest_ps(j,1:2) - p_zero(1,:);
        rest_ps(j,3) = acos(dot(baseline,vect_p)/...
                            norm(vect_p));
    end
    
    rest_ps = sortrows(rest_ps, 3);
    sorted_ps = [];
    for k=1:size(rest_ps,1)
        if(size(sorted_ps, 1) == 0)
            sorted_ps = [rest_ps(k, :)];
        else
            if(sorted_ps(k-1, 3) == rest_ps(k, 3))
                dist1 = norm(sorted_ps(k-1, 1:2) - p_zero);
                dist2 = norm(rest_ps(k, 1:2) - p_zero);
                if(dist1 < dist2)
                    sorted_ps = [sorted_ps; rest_ps(k, :)];
                else
                    temp = sorted_ps(k-1, :);
                    sorted_ps(k-1, :) = rest_ps(k,:);
                    sorted_ps = [sorted_ps; temp];
                end
            else
                sorted_ps = [sorted_ps; rest_ps(k, :)];
            end
        end
    end
    sorted_ps
    size_ps = size(sorted_ps, 1);
    grown = [p_zero;sorted_ps(size_ps,1:2)];
    for n = 1:size_ps-1
        le_size = size(grown)
        v = grown(1,:)-grown(2,:);
        ang = -(acos(dot(baseline, v)/norm(v)));
        pt = sorted_ps(n,1:2)-grown(2,:)
        pt = ([cos(ang), -sin(ang); sin(ang), cos(ang)]*pt')'
        if pt(2) > 0
            grown = [sorted_ps(n, 1:2); grown]
        else
            grown = grown(2:size(grown,1),:)
        end
    end

end
