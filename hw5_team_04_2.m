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
    STRAIGHT_DRIVE = 4;
    KNOCKKNOCK = 5; % then drive in
    
    
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
            if cur_state == STRAIGHT_DRIVE
                cur_state = KNOCKKNOCK;
            else
                cur_state = BUMPED;
            end
        end
        
        if cur_state == SEARCH
            rev = -1*rev;
            SetDriveWheelsCreate(port,0.35+rev,0.35-rev);
            pause(.2);
            SetDriveWheelsCreate(port,0,0);
            [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
            img = imread(str_ip);
            [edges, door_x] = check_door(img);
            if edges > 0
                cur_state = DRIVE2DOOR;
            end
            
        elseif cur_state == SPIN
            % spin in 60 degree intervals and check camera to find a door
            % if no door is found, go back to search
            for i = 1:6
                turnAngle(port, .2, 60);
                [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                img = imread(str_ip);
                [edges, door_x] = check_door(img);
                if edges > 0 % can change this to account for threshold
                    cur_state = DRIVE2DOOR;
                    break;
                else (edges == 0 && i == 6)
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
            [edges, door_x] = check_door(img);
            scale = abs(x-(imgx/2))/(imgx/2);
            if door_x < .9*(imgx/2) %if the door is on the left
                SetDriveWheelsCreate(port,0.2*scale,-0.2*scale); % drive forward and left
            elseif door_x > 1.1*(imgx/2)
                SetDriveWheelsCreate(port,-0.2*scale,0.2*scale); % drive forward and right
            else
                SetDriveWheelsCreate(port,0.2,0.2); % drive straight ahead
                cur_state == STRAIGHT_DRIVE;
            end
            
        elseif cur_state == STRAIGHT_DRIVE
            while true
                [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster,...
                     BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                if (BumpRight)
                    SetDriveWheelsCreate(port,-0.5,-0.5);
                    pause(.3);
                    SetDriveWheelsCreate(port, 0, 0);
                    BeepRoomba(port);
                    BeepRoomba(port);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    turnAngle(port, -45);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    break;
                elseif (BumpLeft)
                    SetDriveWheelsCreate(port,-0.5,-0.5);
                    pause(.3);
                    SetDriveWheelsCreate(port, 0, 0);
                    BeepRoomba(port);
                    BeepRoomba(port);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    turnAngle(port, 45);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    break;
                elseif (BumpFront)
                    SetDriveWheelsCreate(port,-0.5,-0.5);
                    pause(.3);
                    SetDriveWheelsCreate(port, 0, 0);
                    BeepRoomba(port);
                    BeepRoomba(port);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    break;
                end
                
                cur_state = KNOCKKNOCK;
            end
            
%             turnAngle(port, turn_ang);
%             [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
%             img = imread(str_ip);
%             [chance, door_x, door_y] = check_door(img);
%             if chance == 1
%                 cur_state = KNOCKKNOCK;
%             elseif chance == 0
%                 cur_state = SEARCH;
%             else
%                 imgx = size(img, 2);
%                 imgy = size(img, 1);
%                 v1 = [imgx/2, 0] - [imgx/2, imgy];
%                 v2 = [door_x, door_y] - [imgx/2, imgy];
%                 turn_ang = acos(dot(v1, v2));
%             end            
            
        else cur_state == KNOCKKNOCK
            SetDriveWheelsCreate(port,.5,.5);
            while true
                if (BumpRight || BumpLeft || BumpFront)
                    SetDriveWheelsCreate(port,0,0);
                    BeepRoomba(port);
                    BeepRoomba(port);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    SetDriveWheelsCreate(port,-.2,-.2);
                    pause(.4);
                    SetDriveWheelsCreate(port, 0, 0);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    %wait for door to open
                    pause(5);
                    %drive in and stop
                    SetDriveWheelsCreate(port, 0.2, 0.2);
                    pause(3);
                    SetDriveWheelsCreate(port,0,0);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    break;
                end
            end
            break;
        end
        
        pause(.05);
        
        break; %Remove this once infinite loop is ended        
    end
end

%Finds the biggest blob. Takes a binary image as an argument
function mask =  findBiggest(img)
    lbl_mask = bwlabel(img);
    
        A = 0;
        label = 1;
        mask = in_mask;
        while true
            nLabel = lbl_mask == label;
            if(~any(nLabel(:)))
                break;
            end
            A_curr = sum(sum(nLabel));
            if(A_curr > A)
                A = A_curr;
                mask = nLabel;
            end
            label = label + 1;
        end
end

%finds the closest door and edges
function [edges, x_cen] = check_door(img)
    img_hsv = rgb2hsv(img);
    
    maskx = [-1 0 1; -2 0 2; -1 0 1];
    
    %Finding the biggest blue blob
    bh_min = 0.6;
    bh_max = 0.7;
    bs_min = 0.2;
    bs_max = 0.36;
    
    bhh = img_hsv(:, :, 1) > bh_min;
    bhl = img_hsv(:, :, 1) < bh_max;
    
    bh = bhh == bhl;
    
    bsh = img_hsv(:, :, 2) > bs_min;
    bsl = img_hsv(:, :, 2) < bs_max;
    
    bs = bsh == bsl;
    
    b_hs = bh & bs;
    
    pi = bwmorph(b_hs, 'erode', 5);
    pi = bwmorph(pi, 'dilate', 10);
    
    biggest = findBiggest(pi);
    
    %Get vertical edges
    dx = imfilter(img(:, :, 3), maskx);
    vert = im2bw(dx, 0.9);
    
    d_edges = biggest & vert;
    
    [L, edges] = bwlabel(d_edges);
    
    stat = regionprops(L, 'centroid');
    
    
    closest = 0;
    min_d = inf;
    for x = 1:numel(stat)
        if( abs(stat(x).Centroid(1) - size(img, 2)/2) < min_d)
            closest = x;
            min_d = abs(stat(x).Centroid(1) - size(img, 2)/2);
        end
        
        if(stat(x).Centroid(1) == 1 || stat(x).Centroid(1) == size(img,2))
            edges = edges - 1;
        end
    end
    
    x_cen = stat(closest).Centroid(1);
    
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

% imgx = size(img, 2);
% imgy = size(img, 1);
% v1 = [imgx/2, 0] - [imgx/2, imgy];
% v2 = [door_x, door_y] - [imgx/2, imgy];
% turn_ang = acos(dot(v1, v2));