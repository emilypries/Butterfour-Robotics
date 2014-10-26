%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics Fall 2014
%
% Homework 3
%
% Team Number:  4
% Team Leader:  Luis Tolosa (let2120)
% Team Members: Emily Pries (ep2642)
%               Ernesto Sandoval Castillo (es3187) 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
% main function - creates a 2D occupancy grid
% serPort is robot object returned by RoombaInit
function hw3_team_04(serPort)

    % margin of error for position
    err = 0.5;
    % time given to update cell
    timeout = 30;
    
    % space descriptors of robot
    x_pos = 0;
    y_pos = 0;
    r_ang = 0;
    % time descriptors
    t = 0;
    tic;
    
    dir = [0;1];
    
    % states of robot during process
    RAND_DIR = 0;
    RAND_ROT = 1;
    RAND_MOV = 2;
    FIND_OBJ = 3;
    MARK_OBJ = 4;
    RET_PREV = 5;
    
    % initialize occupancy grid
    size = 10;
    cells = zeros(size);
    
    cur_state = RAND_DIR;
    
    while(t < timeout)
        
        [x_pos, y_pos, r_ang] = adjust_dist(serPort, x_pos, y_pos, r_ang);

        t = toc
        
        if (cur_state == RAND_DIR)
            display('State: Choosing direction!!'); 
            %Checks if moves are available
            %t = toc;

            dir = rand(2, 1) * 2 - 1;
            [row, col] = s2g(x_pos + dir(1), y_pos + dir(2), size);
            
            if (~(row < 1 || row > size || col < 1 || col > size) &&...
                 (cells(row, col) == 0))
                cur_state = RAND_ROT;
            end
                         
        elseif (cur_state == RAND_ROT)
            display('State: Rotating to direction!!');
            dir_ang = atan2(dir(2), dir(1));
            turn_ang = (dir_ang - r_ang)*180/pi
            turnAngle(serPort, 0.2, turn_ang);
            [x_pos, y_pos, r_ang] = adjust_dist(serPort, x_pos, y_pos, r_ang);
            cur_state = RAND_MOV;
        elseif (cur_state == RAND_MOV)
            display('State: Moving in direction!!');
            %tic;
            SetDriveWheelsCreate(serPort, 0.1, 0.1);
            cur_state = FIND_OBJ;
        elseif (cur_state == FIND_OBJ)
            display('State: Finding Object!!');
            %t = toc
            [BumpRight, BumpLeft, WheDropRight, WheDropLeft,...
             WheDropCaster, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
            [row, col] = s2g(x_pos, y_pos, size)
            if((row < 1 || row > size || col < 1 || col > size) ||...
                (cells(row, col) == 1))
                cur_state = RET_PREV;
            elseif(BumpRight || BumpLeft || BumpFront)
                SetDriveWheelsCreate(serPort, 0.0, 0.0);
                cur_state = MARK_OBJ;
            end
        elseif(cur_state == MARK_OBJ)
            display('State: Marking the grid!!');
            tic;
            [row, col] = s2g(x_pos, y_pos, size);
            cells(row, col) = 1;
            cur_state = RET_PREV;
        elseif(cur_state == RET_PREV)
            display('State: Returning to previous cell!! :)');
            SetDriveWheelsCreate(serPort, -0.1, -0.1);
            pause(2);
            SetDriveWheelsCreate(serPort, 0.0, 0.0);
            [x_pos, y_pos, r_ang] = adjust_dist(serPort, x_pos, y_pos, r_ang);
            cur_state = RAND_DIR;     
        end 
        
        pause(0.05);
    end

    SetDriveWheelsCreate(serPort, 0.0, 0.0);
    mapObstacles(cells, size);


end


function [x, y, r] = adjust_dist(port, a, b, rad) 
    r = rad + AngleSensorRoomba(port)
    if (r < 0)
        r = r + (2*pi)
    end
    if (r > 2*pi)
        r = r - (2*pi)
    end
    d = DistanceSensorRoomba(port);
    x = a+(d*cos(rad)/0.35) % values will be displayed
    y = b+(d*sin(rad)/0.35)
end

function [row, col] = s2g(xs, ys, size)
    col = (size/2 + 1) + floor(xs);
    row = (size/2) - floor(ys);
end

function mapObstacles(matrix, size)
    f = figure('Visible','on','Position',[100,100,600,600], 'menubar', 'none', 'name', 'Team 4', 'resize', 'off');
    hPlotAxes = axes(...    % Axes for plotting the selected plot
                'Parent', f, ...
                'Units', 'normalized', ...
                'HandleVisibility','callback', ...
                'Position',[0.05 0.05 .8 .8], ...
                 'XLim', [0,size], 'YLim', [0, size], ...
                 'NextPlot', 'add', ...
                 'XGrid', 'on', 'YGrid', 'on');
   for row = 1:size
       for col = 1:size
           if (matrix(row,col) == 1)
               rectangle('Position',[col-1,(size-row),1,1], 'FaceColor','r');
           else
               rectangle('Position',[col-1,(size-row),1,1], 'FaceColor','w');
           end
       end
   end   
   grid on;
end
