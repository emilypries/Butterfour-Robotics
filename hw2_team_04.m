%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics Fall 2014
%
% Homework 2
%
% Team Number:  4
% Team Leader:  Luis Tolosa (let2120)
% Team Members: Emily Pries (ep2642)
%               Ernesto Sandoval Castillo (es3187) 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
% main function - executes bug2 algorithm on iRobot
% serPort is robot object returned by RoombaInit
function hw2_team_04(serPort)
global err;
global goal_dist;
% margin of error used in positional operations and goal distance
err = .05;
goal_dist = 4;

    % states of robot during process
    TO_GOAL    = 0;
    FOLLOW_OBJ = 1;
    
    % space descriptors of the iRobot
    x_pos = 0;
    y_pos = 0;
    r_ang = 0;
    
    xs = [x_pos];
    ys = [y_pos];
  
    % current state and hit-point descriptors
    curr_state = TO_GOAL;
    hit_x_pos = 0;
    hit_y_pos = 0;
    
    % initial robot motion is straight along m-line
    % assumes robot faces goal point
    SetFwdVelRadiusRoomba(serPort, 0.2, inf);
    
    while (true)
        
        % checks if goal is reached
        if (x_pos > goal_dist - err && x_pos < goal_dist + err...
                                    && y_pos > -err && y_pos < err)
            display('goal reached!');
            plot_path(xs, ys)
            break;
        end
        
        [x_pos, y_pos, r_ang] = adjust_dist(serPort, x_pos, y_pos, 0);
        [xs, ys] = save_point(x_pos, y_pos, xs, ys);
        
        % checks for bump on robot
        [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster,...
            BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        
        if (curr_state == TO_GOAL) 
            display('state: heading to goal along the m-line');
            if (BumpFront || BumpLeft || BumpRight)
                hit_x_pos = x_pos
                hit_y_pos = y_pos
                curr_state = FOLLOW_OBJ;
            end
            
        elseif (curr_state == FOLLOW_OBJ)
            display('state: circumnavigating object');
            [x_pos, y_pos, xs, ys] = hw2_follow(serPort, x_pos, y_pos, r_ang,...
                                        hit_x_pos, hit_y_pos, xs, ys);
            % checks if back at m-line and closer to goal
            if ((y_pos > -err && y_pos < err...
                 && (x_pos > hit_x_pos + err || x_pos < hit_x_pos - err)...
                 && ((goal_dist - x_pos) < (goal_dist - hit_x_pos))))
                display('re-encountered the m-line');
            end
            % checks if back at hit-point
            if (x_pos > hit_x_pos - err && x_pos < hit_x_pos + err &&...
                y_pos > hit_y_pos - err && y_pos < hit_y_pos + err)
                display('goal concluded to be unreachable');
                plot_path(xs, ys)
                break;
            end
            
            SetFwdVelRadiusRoomba(serPort, 0.2, inf);
            curr_state = TO_GOAL;
        end
        pause(0.05);
        
    end
    SetFwdVelRadiusRoomba(serPort, 0.0, inf);
end

% controls iRobot to circumvent an object/wall in the way from goal
% takes the args: positional descriptors and hit-point coordinates
% returns: leave-point coordinates
% serPort is robot object returned by RoombaInit
function [lx, ly, xs, ys] = hw2_follow(serPort, x, y, r, hx, hy, xs, ys)
global err;
global goal_dist;
% margin of error used in positional operations
err = .05;
goal_dist = 4;

    % states of robot during process
    BUMP_FOUND  = 0;
    FIND_WALL   = 1;
    FOLLOW_WALL = 2;
    END_WALL    = 3;
    TURN_CORNER = 4;
    END_CORNER  = 5;
    TURN_BACK   = 6;

    % vars to smoothen out robot motion
    timer       = 0;
    crnr_angle  = 0;
    spin_angle  = 0;
    %turn_adjust = 2*pi;
    first_time  = true;
    
    % cur_state tracks current state
    cur_state  = BUMP_FOUND;
    
    while (true)
        
        % updates the timer and space values
        
        [x, y, r] = adjust_dist(serPort, x, y, r);
        [xs, ys] = save_point(x, y, xs, ys);
        hx
        hy
        % checks if robot re-encounters the m-line OR
        % if it reaches the goal OR if it returns to the
        %                           original hit-point
        %
        % allows for error margin of .1 meters
        if ((timer > 15 && y > -err && y < err...
                        && (x > hx + err || x < hx - err)...
                        && ((10 - x) < (10 - hx))) ||... 
            (timer > 15 && x > hx - err && x < hx + err...
                        && y > hy - err && y < hy + err) ||...
            (x > goal_dist - err && x < goal_dist + err && y > -err...
                                 && y < err))
            display('found point of interest');
            break;
        end
    
        % checks for bump on robot
        [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster,...
            BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        
        % checks for wall by robot
        [WallSensor] = WallSensorReadRoomba(serPort);
        
        % starts spinning in place after a bump
        if (cur_state == BUMP_FOUND)
            display('state: bumped')
            SetDriveWheelsCreate(serPort, 0.1, -0.1);
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
            elseif ((timer > ang_time + 2) && (r > spin_angle - (pi/36))...
                    && r < spin_angle + (pi/36))
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
            SetDriveWheelsCreate(serPort, 0.1, 0.1);
            pause(0.5);
            [x, y, r] = adjust_dist(serPort, x, y, r);
            [xs, ys] = save_point(x, y, xs, ys);
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
    
    lx = x;
    ly = y;
    turnAngle(serPort, .075, -r*(180/pi));
    AngleSensorRoomba(serPort);
    [x, y, r] = adjust_dist(serPort, x, y, r)
    [xs, ys] = save_point(x, y, xs, ys);
    
end

% adjust_dist updates space descriptors, avoiding default func. resets
% [a,b] is current (x,y) position
function [x, y, r] = adjust_dist(port, a, b, rad) 
    r = rad + AngleSensorRoomba(port)
    d = DistanceSensorRoomba(port);
    x = a+(d*cos(rad)) % values will be displayed
    y = b+(d*sin(rad))
end

% stores a point's coordinates
function [new_xs, new_ys] = save_point(x, y, old_xs, old_ys)
    new_xs = [old_xs, x];
    new_ys = [old_ys, y];
end

% plots the path the robot took
function plot_path(xs, ys)
    f = figure('Visible','on','Position',[100,100,600,600],'menubar', ...
               'none', 'name', 'Roomba Path: Team 4', 'resize', 'off');
    hPlotAxes = axes(...    % Axes for plotting the selected plot
                'Parent', f, ...
                'Units', 'normalized', ...
                'HandleVisibility','callback', ...
                 'Position',[0.1 0.1 0.8 0.8], ...
                 'XLim', [(min(xs) - .5), (max(xs) + .5)], ... 
                 'YLim', [(min(ys) - .5), (max(ys) + .5)], ...
                 'NextPlot', 'replacechildren', ...
                 'XGrid', 'on', 'YGrid', 'on');
    c = linspace(1,10,length(xs));
    scatter (xs, ys, 20, c, 'filled');
end