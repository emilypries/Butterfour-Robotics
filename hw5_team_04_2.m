%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics Fall 2014
%
% Homework 5
%
% Team Number:  4
% Team Leader:  Luis Tolosa (let2120)
% Team Members: Emily Pries (ep2642)
%               Ernesto Sandoval Castillo (es3187) 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Task #2 - Find, Knock, & Enter CEPSR 6 Door
function hw5_team_04_2(port)
    % Variables
    % Robot spatial descriptors
    xpos = 0;
    ypos = 0;
    ang = 0;
    
    rev = .15;
    
    % States
    SPIN = 0; % spin, find pair of vertical changes 
    SEARCH = 1; % spin failed, move a bit towards blue and then spin again
    DRIVE2DOOR = 2;
    BUMPED = 3; % hit something while searching
    KNOCKKNOCK = 4; % then drive in
    
    cur_state = SPIN;
    
    % Image
    str_ip = 'http://192.168.0.100/snapshot.cgi?user=admin&pwd=&resolution=16&rate=0';
    img = imread(str_ip);
    
    while true
        
        [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
        img = imread(str_ip);
        imgx = size(img, 2);
        imgy = size(img, 1);
        
        [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster,...
                         BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if (BumpRight || BumpLeft || BumpFront)
            cur_state = BUMPED;
        end
        
        if cur_state == SEARCH
            rev = -1*rev;
            SetDriveWheelsCreate(port,0.35+rev,0.35-rev);
            pause(.2);
            SetDriveWheelsCreate(port,0,0);
            [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
            img = imread(str_ip);
            [chance, door_x, door_y] = check_door(img);
            if chance > 0
                imgx = size(img, 2);
                imgy = size(img, 1);
                v1 = [imgx/2, 0] - [imgx/2, imgy];
                v2 = [door_x, door_y] - [imgx/2, imgy];
                turn_ang = acos(dot(v1, v2));
                cur_state = DRIVE2DOOR;
            end
            
        elseif cur_state == SPIN
            % spin in 60 degree intervals and check camera to find a door
            % if no door is found, go back to search
            for i = 1:6
                turnAngle(port, .2, 60);
                [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                img = imread(str_ip);
                [chance, door_x, door_y] = check_door(img);
                if chance > 0 % can change this to account for threshold
                    imgx = size(img, 2);
                    imgy = size(img, 1);
                    v1 = [imgx/2, 0] - [imgx/2, imgy];
                    v2 = [door_x, door_y] - [imgx/2, imgy];
                    turn_ang = acos(dot(v1, v2));
                    cur_state = DRIVE2DOOR;
                    break;
                else (chance == 0 && i == 6)
                    cur_state = SEARCH;
                end
            end
            
        elseif cur_state == BUMPED
            % if we bump, back up, then turn accordingly and go back to
            % searching
            SetDriveWheelsCreate(port,-0.5,-0.5);
            pause(1.5);
            SetDriveWheelsCreate(port, 0, 0);
            [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
            if (BumpLeft || BumpFront)
                turnAngle(port, .2, -90);
            else
                turnAngle(port, .2, 90);
            end
            [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
            
        elseif cur_state == DRIVE2DOOR
            turnAngle(port, turn_ang);
            [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
            img = imread(str_ip);
            [chance, door_x, door_y] = check_door(img);
            if chance == 1
                cur_state = KNOCKKNOCK;
            elseif chance == 0
                cur_state = SEARCH;
            else
                imgx = size(img, 2);
                imgy = size(img, 1);
                v1 = [imgx/2, 0] - [imgx/2, imgy];
                v2 = [door_x, door_y] - [imgx/2, imgy];
                turn_ang = acos(dot(v1, v2));
            end
%             if door_x < (imgx/2) %if the door is on the left
%                 SetDriveWheelsCreate(port,0.2,0.1); % drive forward and left
%             elseif door_x > (imgx/2)
%                 SetDriveWheelsCreate(port,0.1,0.2); % drive forward and right
%             else
%                 SetDriveWheelsCreate(port,0.2,0.2); % drive straight ahead
%             end
%             
            
        else cur_state == KNOCKKNOCK
            % knock twice
            for i = 1:2
                SetDriveWheelsCreate(port,0.5,0.5);
                while true
                    [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster,...
                         BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                    if (BumpRight || BumpLeft || BumpFront)
                        SetDriveWheelsCreate(port,-0.5,-0.5);
                        pause(.3);
                        SetDriveWheelsCreate(port, 0, 0);
                        [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                        BeepRoomba(port);
                        BeepRoomba(port);
                        break;
                    end
                end
            end
            % wait 3 seconds for door to open, then drive in
            pause(3);
            SetDriveWheelsCreate(port, 0.5, 0.5);
            pause(2);
            SetDriveWheelsCreate(port,0,0);
            break;
        end
        
        pause(.05);
        
        break; %Remove this once infinite loop is ended        
    end
end

function [chance, door_x, door_y] = check_door(raw_image)
    % Check the raw_image to find find doors
    % If no door is found, set chance to 0
    
end


% updates the spatial descriptors
function [x, y, r] = adjust_dist(port, a, b, rad) 
    r = rad + AngleSensorRoomba(port);
    if (r < 0)
        r = r + (2*pi);
    end
    if (r > 2*pi)
        r = r - (2*pi);
    end
    d = DistanceSensorRoomba(port)/.35;
    x = a+d*cos(r); % values will be displayed
    y = b+d*sin(r);
end