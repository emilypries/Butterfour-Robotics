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
function hw4_path_planner(obj_file, pts_file)
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
        grownmatrix = [grownmatrix; (obj_v)];
        rows = rows + supernums(i);
    end
    
    %Run dijkstra
    dijkstra_matrix = grownmatrix;
    obstindex = [];
    [gmlength, gmwidth] = size(grownmatrix);
    [snlength, snwidth] = size(supernums);
    n = 2;
    g = 1;
    for j = 1:snwidth
        n = n+1;
        for i = 1:supernums(j)
            obstindex = [obstindex; n];
        end
    end
    supernums
    grownmatrix
    obstindex
    dijkstra_matrix = [grownmatrix, obstindex];
    dijkstra_matrix = [start, 1; goal, 2; dijkstra_matrix];
    dijkstra_matrix
    [dlength, dwidth] = size(dijkstra_matrix);
    % Make a matrix where each row contains the list of vertices in an
    % object
    obj_array = {};
    i = 1;
    while true
        objrow = [];
        for j = i:dlength
            if (dijkstra_matrix(i,3) == dijkstra_matrix(j,3))
                if ~ismember(i,objrow)
                    objrow = [objrow, i];
                end
                if ~ismember(j,objrow)
                    objrow = [objrow, j];
                end
            end

        end

        obj_array = [obj_array, objrow];
        i = i + size(objrow,2);
        if i > dlength
            break;
        end
    end 
    % go thruogh each row of the obj_array
    %check if each edge in any edge in each object crosses an edge in
    %another object
    hull_array = cell(1,size(obj_array,2));
    label = (1:size(obj_array,2));
    label
    supermatches = cell(1, size(obj_array,2));

    for i = 1:size(obj_array,2)
        if i < 3
            continue;
        end

        matches = [i];
        for j = 1:size(obj_array{i},2)
            a = j;
            b = a+1;
            if b > size(obj_array{i},2)
                b = 1;
            end
            a = obj_array{i}(a);
            b = obj_array{i}(b); 
            % right now we have an edge

            for k = i+1:size(obj_array,2)
                if (label(i) == label(k))
                    continue;
                end
                for l = 1:size(obj_array{k},2)
                    c = l;
                    d = c+1;
                    if d > size(obj_array{k},2)
                        d = 1;
                    end
                    c = obj_array{k}(c);
                    d = obj_array{k}(d); 
                    disp(dijkstra_matrix)
                    if check_cross(dijkstra_matrix(a,1), dijkstra_matrix(a,2), dijkstra_matrix(b,1), dijkstra_matrix(b,2),dijkstra_matrix(c,1), dijkstra_matrix(c,2), dijkstra_matrix(d,1), dijkstra_matrix(d,2))               
                        matches = [matches, k];
                        label(i) = label(k);
                        break;
                    end
                end
            end
        end 
        if size(matches) == 0
            supermatches{i} = i;
        else
            supermatches{i} = matches;
        end       
    end
    supermatches
    disp('abc')
    %supermatches{7}
    %supermatches{8}
    for s = 3:size(supermatches,2)-1
        n = 1;
        while n <= size(supermatches{s},2)
            %n
%           if s == 5 
%               disp (' i think size is:')
%               disp(size(supermatches{s},2))
%           end
            %check that no other supermatches contains it
            if supermatches{s}(n) == 0
                n = n+1;
                continue;
            end

            for p = s+1:size(supermatches,2)

                for r = 1:size(supermatches{p},2)
                    if size(supermatches{p},2) == 0
                        continue;
                    end

                    if supermatches{p}(r) == 0
                        continue;
                    end
                    disp('now comparing: ' )
                    supermatches{s}(n)
                    supermatches{p}(r)

                    if supermatches{s}(n) == supermatches{p}(r)

                        disp('found a match between: ')
                        s
                        p
                        disp('supermatches s is:')
                            supermatches{s}
                            disp('supermatches p is:')
                            supermatches{p}
                        for q = 1:size(supermatches{p},2)
                        %add r to supermatches if it is not already there
                            disp('add?')
                            supermatches{p}(q)

                            if ~(ismember(supermatches{p}(q),supermatches{s}))
                                disp('added')
                                supermatches{s} = [supermatches{s}, supermatches{p}(q)];

                            end
                        end

                        supermatches{p} = [0];
                        disp(supermatches{p})
                    end
                    if supermatches{p} == [0]
                        disp('breaking')
                        break;
                    end
                end
            end
            
            disp ('end of while')
            n = n+1;
                
        end
        disp('out of while')
        supermatches{s}
    end
    disp('here is sm')
    supermatches       
%   134 2 34 45
%   sm: 1345 2 0 0
%   ha: 1


    hull_array = cell(1, size(supermatches,2));

    for s = 3:size(supermatches,2)
        for n = 1:size(supermatches{s},2)
            disp('size of this bucket is:')
            disp (size(supermatches{s},2))
            if supermatches{s}(n) == 0
                break;
            end
            hull_array{s}
            obj_array{supermatches{s}(n)}
            hull_array{s} = [hull_array{s}, obj_array{supermatches{s}(n)}];
            hull_array{s}
        end
        hull_array{s}
    end      

    hull_array

    % with the hull array of indices
    % make cell array of points]
    hull_points = cell(1,size(obj_array,2));
    disp('size of obj_array,2')
    for i = 1:size(obj_array,2)
        disp(i)
        if size(hull_array{i},2) == 0
            disp('empty obj')
            continue;
        end
        size(hull_array{i},2)
        for j = 1:size(hull_array{i},2)
            disp('converting from object:')
            disp(i)
            tempi = hull_array{i}(j);
            hull_points{i}
            hull_points{i} = [hull_points{i}; dijkstra_matrix(tempi,1), dijkstra_matrix(tempi,2)];
            hull_points{i}
        end
    end
    hull_points
    disp('the pointss')
    hull_points{3}
    hull_points{4}
    hull_points{5}
    hull_points{11}


    % pass each cell to convex hull
    real_dijkstra = [];
    real_dijkstra = [start, 1; goal, 2];
    objcount = 3;
    for i = 1:size(hull_points,2)
        if (size(hull_points{i},1) == 0)
            continue;
        end
        convex_return = [convex_hull(hull_points{i}(:,1:2))];
        convex_return = [convex_return, ones(size(convex_return,1),1)*objcount];
        objcount = objcount+1;
        real_dijkstra = [real_dijkstra; convex_return];
    end
    real_points = size(real_dijkstra,1);
        
    for w = 1:size(vwall,1)
        real_dijkstra = [real_dijkstra; vwall(w,1), vwall(w,2), objcount];
    end

    [path, path_dist, dist_array] = dijkstra(real_dijkstra, real_points);

    [palength, pawidth] = size(real_dijkstra);
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
    for i = 1:real_points
        for j = i:real_points
            if (dist_array(i,j) ~= inf)
                %TODO: draw line from pa(i,1) pa(i,2) to pa(j,1) pa (j,2)
                %disp('didnt draw:' + i + j)
                plot([real_dijkstra(i,1), real_dijkstra(j,1)], [real_dijkstra(i,2), real_dijkstra(j,2)], 'k');
            end
        end
    end
    % Draw the Path
    [pathrow, pathlength] = size(path);

    for i = 1:(pathrow-1)
        disp('drew path edge')
        plot([path(i, 1), path(i+1,1)], [path(i, 2), path(i+1,2)], 'r');
    end
    plot(start(1), start(2), '+', 'markersize', 13);
    plot(goal(1), goal(2), '+', 'markersize', 13);
    plot(real_dijkstra(:,1),real_dijkstra(:,2),'o');
    plot(hull_points{5}(:,1),hull_points{5}(:,2),'r*');
    camroll(90);

end

% grows an obstacle given the object's vertex coordinates
function grown = convex_hull(coords)
    % find the rightmost, lowest point of object, p_zero
    p_zero = coords(1,:);
    for i=2:size(coords,1)
        new_p = coords(i,:);
        if ((new_p(2) == p_zero(2) && new_p(1) >= p_zero(1)) || new_p(2) < p_zero(2))
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
    
    rest_ps = sortrows(rest_ps, 3)
        
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
        
    size_ps = size(sorted_ps, 1)
    grown = [p_zero;sorted_ps(size_ps,1:2)];
    n = 1;
    while n < size_ps-1
        le_size = size(grown)
        v1 = grown(1,:)-grown(2,:);
        disp('v1 is')
        v1 = [v1 0]
        pt = sorted_ps(n,1:2)-grown(1,:) % should this be grown(1,:) or 2
        disp('v2 is')
        pt = [pt 0]
            
        z = cross(v1, pt)
        z(3)

        if z(3) > 0
            grown = [sorted_ps(n, 1:2); grown]
            n = n + 1;
        else
            disp('Popped from stack')
            %n = n + 1;
            grown(1,:)
            grown = grown(2:size(grown,1),:)
        end


    end

end    
    
% different take on convex hull, unused
function grown = convex_hull2(coords)
        n = size(coords, 1);
        tempmin = inf;
        tempx = inf;
        tempi = 0;
        for i=1:n
            if coords(i,2) < tempmin
                tempi = i;
                tempx = coords(i,1)
                tempmin = coords(i,2); 
            elseif coords(i,2) == tempmin
                if coords(i,1) > tempx
                    tempi = i;
                    tempx = coords(i,1)
                    tempmin = coords(i,2); 
                end
            end
        end
        tempp = coords(1,:);
        coords(1,:) = coords(tempi,:);
        coords(tempi,:) = tempp;
        basev = [1 0];
        new_coords = [coords, zeros(size(coords,1),2)]
        for i=2:n
            vect = coords(i,1:2) - coords(1,1:2);
            new_coords(i,3) = acos(vect(1)/norm(vect));
            new_coords(i,4) = norm(vect);
        end
        
        sorted_coords = [new_coords(1,:)];
        for k=2:size(new_coords,1)
            if(size(sorted_coords, 1) == 1)
                sorted_coords = [sorted_coords; new_coords(k, :)];
            else
                if(new_coords(k,3) > sorted_coords(k-1,3))
                    sorted_coords = [sorted_coords; new_coords(k, :)];
                elseif (new_coords(k,3) < sorted_coords(k-1,3))
                    n = k-2;
                    
                    while n > 1
                        if(new_coords(k,3) > sorted_coords(n,3))
                            break;
                        elseif (new_coords(k,3) == sorted_coords(n,3))
            
                            if new_coords(k,4) > sorted_coords(n,4)
                                break;
                            end 
                        end
                        n = n-1;
                    end
                    
                    sorted_coords = [sorted_coords(1:n,:); new_coords(k,:); sorted_coords(n+1:size(sorted_coords,1), :)];
                    
                elseif(new_coords(k, 3) == sorted_coords(k-1, 3))
                    n = k-1;
                    while true
                        if(new_coords(k, 3) ~= sorted_coords(n,3))
                            break;
                        end
                        if new_coords(k,4) > sorted_coords(n,4)
                            break;
                        else
                            n = n - 1;
                        end
                    end
                    if n == k-1
                        sorted_coords = [sorted_coords; new_coords(k,:)];
                    else
                        sorted_coords = [sorted_coords(1:n,:); new_coords(k,:); sorted_coords(n+1:size(sorted_coords,1), :)];
                    end
                end
            end
        end
        
        grown = [sorted_coords(1,1:2);sorted_coords(size(sorted_coords,1),1:2)];
        n = 2;
        while n < size(sorted_coords,1)
            le_size = size(grown)
            v1 = grown(1,:)-grown(2,:);
            disp('v1 is')
            v1 = [v1 0]
            pt = sorted_coords(n,1:2)-grown(1,:) % should this be grown(1,:) or 2
            disp('v2 is')
            pt = [pt 0]
            
            z = cross(v1, pt)
            z(3)

            if z(3) > 0
                grown = [sorted_coords(n, 1:2); grown]
                n = n + 1;
            else
                disp('Popped from stack')
                %n = n + 1;
                grown(1,:)
                grown = grown(2:size(grown,1),:)
            end
        end
        
        
    end
    
    function h = ccw(p1, p2, p3)
        h = (p2(1) - p1(1))*(p3(2) - p1(2)) - (p2(2) - p1(2))*(p3(1) - p1(1))
    end

    function [path, currdist, dist_array] = dijkstra(pa, num_obj_points)
% need array of points including vertices of walls
%how many points are obj points and how many are walls  

   % pa = [0,0;6,6;1,1;1,0;0,1;2,2;3,3;4,4;5,5];
    %[palength, pawidth] = size(pa);
    palength = num_obj_points;
    dist_array = calc_all_dists(pa, num_obj_points);
    
    % Dijkstra's Algorithm
    % Initialize Variables:
    visited = zeros(1,palength);
    distances = inf(1,palength);
    bestway = zeros(1,palength);
    currdist = 0;
    
    % Set up Cycle 1
    distances(1) = 0;
    current = 1;
    path = [1];
    
    while true
        % Mark current vertex as visited
        visited(current) = 1;
        
        % Update best distances so far
        for i = 1:palength
            if (visited(i)==0 && distances(i)> (currdist + dist_array(i,current)))
                distances(i) = currdist + dist_array(i,current);
                bestway(i) = current;
            end
        end
        
        % If we have reached the path or if we have just visited the goal,
        % break
        if(all(visited  == visited(1)))
            disp('visited all nodes')
            break;
        end
        
        % Pick the next vertex with the minimum distance
        tmin = inf;
        tcurr = 0;
        for i = 1:palength
            if visited(i)==1
                continue
            end
            if (distances(i)<tmin && distances(i)~=0 && distances(i) ~= inf)
                tmin = distances(i);
                tcurr = i;
            end
        end
        if tcurr == 0
            disp('did not find a min')
            break;
        end
        
        % If we got to the goal, break
        if (tcurr == 2)
            disp('found goal')
            break;
        end
        
        if (dist_array(current,2) < inf)
            currdist = currdist + dist_array(current,2);
            path = [path, 2];
            visited(2)=1;
            disp('reached the goal')
            break;
        end
        
        %Update conditions for next loop
        current = tcurr;
        currdist = currdist + tmin;
        path = [path, current];
    end
    path = [];
    if (visited(2)==1)
        %visited2 = zeros(1,palength);
        path = [];
        currfrom = 2;
        while currfrom ~= 1
            %visited2(currfrom) = 1;
            path = [currfrom, path]
            currfrom = bestway(currfrom)
        end
        path = [1, path];
    end
    path = [pa(path,1), pa(path,2)];
end

function dist_array2 = calc_all_dists(point_array, num_obj_points)
    % dist_array creates a matrix with the distances between all vertices.
    % If a vertex is unreachable from another or is connecting to itself,
    % the edge distance is Inf
    
    % Initialize arrays
    %[palength, pawidth] = size(point_array);
    palength = num_obj_points;
    dist_array = [];
    dist_row = [];
    
    % Build the raw array (calculate distances for all edges regardless of
    % crossing)
    for i = 1:palength
        for j = 1:palength
            % If a vertex is connect to itself, the distance is infinite
            if i == j
                dist_row = [dist_row, inf];
            % Otherwise, calculate the distance
            else
            dist_row = [dist_row, get_dist(point_array(i,1), point_array(i,2), point_array(j,1), point_array(j,2))];
            end
        end
        dist_array = [dist_array; dist_row];
        dist_row = [];
    end
    
   % disp(dist_array)
   
   dist_array2 = dist_array;
   crosslist = [];
   
   % Find internal edges within obstacles MOVE UP TO FIRST ROUND
   for i = 1:palength
       for j = i+1:palength
           if (point_array(i,3) == point_array(j,3) && (j-i ~= 1 && j-i ~= (j-point_array(j,3))))
               dist_array2(j,i) = inf;
               dist_array2(i,j) = inf;
               crosslist = [crosslist; i,j];
           end
       end
   end
   total_num = size(point_array,1);
   i = num_obj_points+1;
   disp('pre-wall')
   while i <= total_num
       j = i + 1;
       if j > total_num
           j = num_obj_points+1;
       end
       crosslist = [crosslist; i,j];
       i = i+1;
       
   end
   %more invalid edges:
   
    [crosslength, crosswidth] = size(crosslist);
    % Check which edges cross internals
    disp ('started checking cross')
    disp size
    disp(palength)
    perr = .4;
    for i = 1:palength
        for j = i:palength
            
            if ((point_array(i,1) > 1.5367-perr && point_array(i,1) < 1.5367+perr)  && (point_array(i,2) > -2.5860 - perr && point_array(i,2) < -2.5860 + perr ))
                dist_array2(i,:) = inf;
                dist_array2(:,i) = inf;
            end
            if ((point_array(j,1) > 1.5367-perr && point_array(j,1) < 1.5367+perr)  && (point_array(j,2) > -2.5860 - perr && point_array(j,2) < -2.5860 + perr ))
                dist_array2(j,:) = inf;
                dist_array2(:,j) = inf;
            end
            for k = 1:crosslength        
                    if check_cross(point_array(i,1), point_array(i,2), point_array(j,1), point_array(j,2), point_array(crosslist(k,1),1), point_array(crosslist(k,1),2), point_array(crosslist(k,2),1), point_array(crosslist(k,2),2))
                        dist_array2(i,j) = inf;
                        dist_array2(j,i) = inf;
                    end
            end
        end
    end    
end
    
function dist = get_dist(x1, y1, x2, y2)
    % get_dist calculates the distance between two points
    dist = sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1)));
end

function isit = check_cross(Ax, Ay, Bx, By, Cx, Cy, Dx, Dy)
    % check_cross returns true if two line segments cross by calculating
    % the dot product and crossing point (if one exists), then checking to
    % see if the crossing point is on the line segments
    a = [Ax, Ay];
    c = [Cx, Cy];
    e = [Bx - Ax, By - Ay];
    f = [Dx - Cx, Dy - Cy]; 
    p = [-e(2), e(1)];
    if (dot(f,p) == 0)
        isit = false;
        return;
    end
    h = ( dot((a - c),p) )/( dot(f,p) );
    if (h < 0 || h > 1)
        isit = false;
        return;
    end
    cross = c+f*h;
    crx = cross(1);
    cry = cross(2);       
    %if (((((crx<=Ax) && (crx>=Bx)) || ((crx<=Bx) && (crx >= Ax))) && (((crx<=Cx) && (crx>=Dx)) || ((crx<=Dx) && (crx>=Cx)))) && ((((cry<=Ay) && (cry>=By)) || ((cry<=By) && (cry >= Ay))) && (((cry<=Cy) && (cry>=Dy)) || ((cry<=Dy) && (cry>=Cy)))))
    if (((((crx<Ax) && (crx>Bx)) || ((crx<Bx) && (crx > Ax))) && (((crx<Cx) && (crx>Dx)) || ((crx<Dx) && (crx>Cx)))) && ((((cry<Ay) && (cry>By)) || ((cry<By) && (cry > Ay))) && (((cry<Cy) && (cry>Dy)) || ((cry<Dy) && (cry>Cy)))))
        isit = true;
    else
        isit = false;
    end
end


    