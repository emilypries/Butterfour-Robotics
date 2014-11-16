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

function dijkstra
    %dijkstra(pa,start,goal)
    %need to add start and goal to array)
    %index 1 = start
    %index 2 = goal
    
    pa = [0,0;6,6;1,1;1,0;0,1;2,2;3,3;4,4;5,5];
    plot(pa(:,1), pa(:,2))
    [palength, pawidth] = size(pa);
    dist_array = calc_all_dists(pa);
    visited = zeros(1,palength);
    distances = inf(1,palength);
    %first, set the first cycle
    distances(1) = 0;
    visited(1) = 1;
    current = 1;
    currdist = 0;
    path = [1];
    while true
        visited(current) = 1;
        for i = 1:palength %update distances
            if (visited(i)==0 && distances(i)> (currdist + dist_array(i,current)))
                distances(i) = currdist + dist_array(i,current);
            end
        end
        
        if((all(visited  == visited(1))) || visited(2)==1)
            break;
        end
        tmin = inf;
        tcurr = 0;
        for i = 1:palength
            if (visited(i) == 0 && distances(i)<tmin)
                tmin = distances(i);
                tcurr = i;
            end
        end
        if (tcurr == 0)
            break;
        end
        current = tcurr;
        currdist = currdist + tmin;
        path = [path, current];
    end
    disp (path)
end

function dist_array = calc_all_dists(point_array)
    %create nxn matrix of points
    %atoa atob atoc atod atoe
    %btoa btob btoc btod btoe
    
    [palength, pawidth] = size(point_array);
    dist_array = [];
    dist_row = [];
    for i = 1:palength
        for j = 1:palength
            if i == j
                dist_row = [dist_row, inf];
            else
            dist_row = [dist_row, get_dist(point_array(i,1), point_array(i,2), point_array(j,1), point_array(j,2))];
            end
        end
        dist_array = [dist_array; dist_row];
        dist_row = [];
    end
    disp(dist_array)
    dist_array2 = dist_array;
    for i = 1:palength
        for j = 1:palength
            for k = 1:palength
                for l = 1:palength
                    if check_cross(point_array(i,1), point_array(i,2), point_array(j,1), point_array(j,2), point_array(k,1), point_array(k,2), point_array(l,1), point_array(l,2))
                        dist_array2(i,j) = inf;
                        dist_array2(j,i) = inf;
                        dist_array2(k,l) = inf;
                        dist_array2(l,k) = inf;
                    end
                end
            end
        end
        end
    disp new:
    disp(dist_array2)
    %cool now we have the distance array

    
end
    
function dist = get_dist(x1, y1, x2, y2)
    dist = sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1)));
end

function isit = check_cross(Ax, Ay, Bx, By, Cx, Cy, Dx, Dy)
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

