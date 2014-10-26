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
    timeout = 60;
    
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
    global size;
    size = 16;
    global cells;
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
            [row, col] = s2g(x_pos + 1.5*dir(1), y_pos + 1.5*dir(2), size);
            
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
            SetDriveWheelsCreate(serPort, 0.2, 0.2);
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
            [x_pos, y_pos, r_ang] = hw3_follow(serPort, x_pos, y_pos, r_ang);
            tic;
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
<<<<<<< HEAD
                'Position',[.05 .05 0.8 0.8], ...
=======
                'Position',[0.05 0.05 .8 .8], ...
>>>>>>> origin/master
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


% controls iRobot to circumvent an object/wall in the way from goal
% takes the args: positional descriptors and hit-point coordinates
% returns: leave-point coordinates
% serPort is robot object returned by RoombaInit
function [x, y, r] = hw3_follow(serPort, x, y, r)
global cells;
global size;
global err;
% margin of error used in positional operations
err = .15;

    % states of robot during process
    BUMP_FOUND  = 0;
    FIND_WALL   = 1;
    FOLLOW_WALL = 2;
    END_WALL    = 3;
    TURN_CORNER = 4;
    END_CORNER  = 5;

    % vars to smoothen out robot motion
    timer       = 0;
    crnr_angle  = 0;
    spin_angle  = 0;
    first_time  = true;
    
    mark_r = [];
    mark_c = [];
    
    hx = x;
    hy = y;
    
    % cur_state tracks current state
    cur_state  = BUMP_FOUND;
    
    while (true)
        
        % updates the space values        
        [x, y, r] = adjust_dist(serPort, x, y, r);
        
        % checks if robot re-encounters the m-line OR
        % if it reaches the goal OR if it returns to the
        %                           original hit-point
        %
        % allows for error margin of .1 meters
        if ((timer > 15 && x > hx - err && x < hx + err...
                        && y > hy - err && y < hy + err))
            display('Back at hit point');
            break;
        end
        
        [row, col] = s2g(x, y, size);
        if(cells(row,col) == 1)
            display('Already marked');
            break;
        end
        
        mark_r = [mark_r row];
        mark_c = [mark_c col];
        
        % checks for bump on robot
        [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster,...
            BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        
        % checks for wall by robot
        [WallSensor] = WallSensorReadRoomba(serPort);
        
        % starts spinning in place after a bump
        if (cur_state == BUMP_FOUND)
            display('state: bumped')
            SetDriveWheelsCreate(serPort, 0.09, -0.09);
            cur_state = FIND_WALL;
            spin_angle = r;
            if (first_time)
                tic;
                first_time = false;
            end
            ang_time = toc;
            
        % stops spinnning when wall is found
        %
        % if wall not found, spins back a little 
        % to try again until wall is found
        elseif (cur_state == FIND_WALL)
            display('state: spinning in place');
            if (WallSensor && ~BumpRight && ~BumpFront)
                cur_state = FOLLOW_WALL;
                SetDriveWheelsCreate(serPort, 0, 0);
            elseif ((timer > ang_time + 2) && (r > spin_angle - (pi/18))...
                    && (r < spin_angle + (pi/18)))
                cur_state = TURN_CORNER;
            end      
            
        % robot moves straight along wall
        elseif (cur_state == FOLLOW_WALL)
            display('state: following wall');
            SetFwdVelRadiusRoomba(serPort, 0.2, inf);
            cur_state = END_WALL;
            
        % goes straight and reacts to either new bump
        % or losing followed wall at a corner turn
        elseif (cur_state == END_WALL)
            display('state: tracking wall');
            if (BumpRight || BumpFront || BumpLeft)
                cur_state = BUMP_FOUND;
            elseif (~WallSensor)
                cur_state = TURN_CORNER;
            end
            
        % robot goes fwd and spins about right wheel when wall is lost
        elseif (cur_state == TURN_CORNER)
            display('state: relocating wall / turning corner');
            SetDriveWheelsCreate(serPort, 0.075, 0.075);
            pause(0.7);
            [x, y, r] = adjust_dist(serPort, x, y, r);
            SetDriveWheelsCreate(serPort, 0, 0.1);
            cur_state = END_CORNER;
            crnr_angle = r;
            
        % either bumps anew or detects new wall
        elseif (cur_state == END_CORNER)
            display('state: relocated wall');
            if (WallSensor)
                cur_state = FOLLOW_WALL;
            elseif (BumpFront || (BumpRight && (r < (crnr_angle+(pi/12)))))
                cur_state = BUMP_FOUND;
            end
        end    
    
        pause(0.05);
        timer = toc;
       
    end
    
    SetDriveWheelsCreate(serPort, 0.0, 0.0);
    [x, y, r] = adjust_dist(serPort, x, y, r)
    
    n = length(mark_r);
    
    for i = 1:n
        if(mark_r(i) <= size && mark_r(i) > 0 && mark_c(i) > 0 &&...
                mark_c(i) <= size)
            cells(mark_r(i), mark_c(i)) = 1;
        end
    end
    
    turnAngle(serPort, 0.2, -90);
    [x, y, r] = adjust_dist(serPort, x, y, r);
    SetDriveWheelsCreate(serPort, -0.2, -0.2);
    pause(1);
    [x, y, r] = adjust_dist(serPort, x, y, r);
    
end
