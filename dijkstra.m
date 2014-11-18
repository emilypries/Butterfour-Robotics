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
    
% calculates shortest path using dijkstra's algorithm

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
   crosslist
   
    [crosslength, crosswidth] = size(crosslist);
    % Check which edges cross internals
    disp ('started checking cross')
    disp size
    disp(palength)
    for i = 1:palength
        for j = i:palength
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

