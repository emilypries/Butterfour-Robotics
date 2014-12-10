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

%     \/         
%    <'l        
%     ll         
%     llama~
%     || ||
%     '' ''

% Task #2 - Find, Knock, & Enter CEPSR 6 Door
% 'port' is robot object returned by RoombaInit()
function hw5_team_04_2(port)
    % Variables
    % Robot spatial descriptors
    xpos = 0;
    ypos = 0;
    ang = 0;
    
    rev = .01;
    
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
                         BumpFront] = BumpsWheelDropsSensorsRoomba(port);
        if (BumpRight || BumpLeft || BumpFront)
            if cur_state == STRAIGHT_DRIVE
                cur_state = KNOCKKNOCK;
            else
                cur_state = BUMPED;
            end
        end
        
        if cur_state == SEARCH
            display('state: searching');
            rev = -1*rev;
            SetDriveWheelsCreate(port,0.04+rev,0.04-rev);
            pause(.4);
            SetDriveWheelsCreate(port,0,0);
            [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
            img = imread(str_ip);
            [edges, door_x] = check_door(img);
            if edges > 0
                cur_state = DRIVE2DOOR;
            end
            
        elseif cur_state == SPIN
            display('state: spinning');
            % spin in 60 degree intervals and check camera to find a door
            % if no door is found, go back to search
            for i = 1:6  
                %[xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                img = imread(str_ip);
                [edges, door_x] = check_door(img);
                if edges > 0 % can change this to account for threshold
                    cur_state = DRIVE2DOOR;
                    break;
                else (edges == 0 && i == 6)
                    cur_state = SEARCH;
                end
                turnAngle(port, .04, 60);
            end
            
        elseif cur_state == BUMPED
            display('state: bumped');
            % if we bump, back up, then turn accordingly and go back to
            % searching
            SetDriveWheelsCreate(port,-0.05,-0.05);
            pause(1.5);
            SetDriveWheelsCreate(port, 0, 0);
            [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
            if (BumpLeft || BumpFront)
                turnAngle(port, .03, -90);
            else
                turnAngle(port, .03, 90);
            end
            [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
            
        elseif cur_state == DRIVE2DOOR
            display('state: driving to door');
            [edges, door_x] = check_door(img);
            scale = abs(door_x-(imgx/2))/(imgx/2);
            if door_x < .95*(imgx/2) %if the door is on the left
                SetDriveWheelsCreate(port,0.02*scale,-0.02*scale); % drive forward and left
            elseif door_x > 1.05*(imgx/2)
                SetDriveWheelsCreate(port,-0.02*scale,0.02*scale); % drive forward and right
            else
                SetDriveWheelsCreate(port,0.05,0.05); % drive straight ahead
                cur_state == STRAIGHT_DRIVE;
            end
            
        elseif cur_state == STRAIGHT_DRIVE
            display('state: driving straight');
            while true
                [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster,...
                     BumpFront] = BumpsWheelDropsSensorsRoomba(port);
                if (BumpRight)
                    SetDriveWheelsCreate(port,-0.05,-0.05);
                    pause(.3);
                    SetDriveWheelsCreate(port, 0, 0);
                    BeepRoomba(port);
                    BeepRoomba(port);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    turnAngle(port, -45);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    break;
                elseif (BumpLeft)
                    SetDriveWheelsCreate(port,-0.05,-0.05);
                    pause(.3);
                    SetDriveWheelsCreate(port, 0, 0);
                    BeepRoomba(port);
                    BeepRoomba(port);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    turnAngle(port, 45);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    break;
                elseif (BumpFront)
                    SetDriveWheelsCreate(port,-0.05,-0.05);
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
            display('state: knocking');
            SetDriveWheelsCreate(port,.05,.05);
            while true
                if (BumpRight || BumpLeft || BumpFront)
                    SetDriveWheelsCreate(port,0,0);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    SetDriveWheelsCreate(port,-.05,-.05);
                    pause(.4);
                    SetDriveWheelsCreate(port, 0, 0);
                    BeepRoomba(port);
                    BeepRoomba(port);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    %wait for door to open
                    pause(5);
                    %drive in and stop
                    SetDriveWheelsCreate(port, 0.05, .05);
                    pause(3);
                    SetDriveWheelsCreate(port,0,0);
                    [xpos, ypos, ang] = adjust_dist(port, xpos, ypos, ang);
                    break;
                end
            end
            break;
        end
        
        pause(.05);       
    end
end

%Finds the biggest blob. Takes a binary image as an argument
function mask =  findBiggest(img)
    [lbl_mask,n] = bwlabel(img);
    %imshow(lbl_mask);
    A = 0;
    mask = [];
    for label = 1:n
        nLabel = lbl_mask == label;
        if(~any(nLabel(:)))
            continue;
        end
        A_curr = sum(sum(nLabel));
        if(A_curr > A)
            A = A_curr;
            mask = nLabel;
        end
    end
        
end

%finds the closest door and edges
function [edges, x_cen] = check_door(img)
    img_hsv = rgb2hsv(img);
    
    maskx = [-1 0 1; -2 0 2; -1 0 1];
    
    %Finding the biggest blue blob
    bh_min = 0.5;
    bh_max = 0.65;
    bs_min = 0.09;
    bs_max = 0.2;
    
    bhh = img_hsv(:, :, 1) > bh_min;
    bhl = img_hsv(:, :, 1) < bh_max;
    
    bh = bhh == bhl;
    
    bsh = img_hsv(:, :, 2) > bs_min;
    bsl = img_hsv(:, :, 2) < bs_max;
    
    bs = bsh == bsl;
    
    b_hs = bh & bs;
    
    pi = bwmorph(b_hs, 'erode', 5);
    pi = bwmorph(pi, 'dilate', 10);

    %fig = figure;
    %hold on;
    %imshow(pi);
    biggest = findBiggest(pi);
    
    if size(biggest,1) == 0
        edges = 0;
        x_cen = 0;
        return;
    end
    %imshow(biggest);
    %Get vertical edges
    dx = imfilter(img(:, :, 3), maskx);
    vert = im2bw(dx, 0.75);
    vert = bwmorph(vert, 'dilate', 3);
    %imshow(vert);
    
    d_edges = biggest & vert;
    %imshow(d_edges);
    %hold off;
    [L, edges] = bwlabel(d_edges);
    
    stat = regionprops(L, 'centroid');
    
    closest = 0;
    min_d = inf;
    if size(stat,1) == 0
        edges = 0;
        x_cen = 0;
        return;
    end
    for x = 1:size(stat,1)
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