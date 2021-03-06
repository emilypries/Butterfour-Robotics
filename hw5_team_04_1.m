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

% Task #1 - Color Marker Tracking
% 'port' is robot object returned by RoombaInit()
function hw5_team_04_1(port)
    err = 5000;
    % spatial descriptors of robot
    xpos = 0;
    ypos = 0;
    ang  = 0;
    % states of robot during process
    FIND_OBJ = 0;
    TURN2OBJ = 1;
    MOVE_FWD = 2;
    cur_state = FIND_OBJ;
    
    % will get an initial snapshot of the camera and a single
    % input from the user which will decide the target color
    str_ip = 'http://192.168.0.101/snapshot.cgi?user=admin&pwd=&resolution=16&rate=0';
    img = imread(str_ip);
    axis = size(img,2)/2;
    turn_left = true;
    fh = figure; imshow(img);
    h = imrect;
    rect = round(h.getPosition);
    delete(fh);
    
    %Extract values from rect
    width = rect(3);
    height = rect(4);

    %Gets target color, CURRENTLY RGB 
    img_hsv = rgb2hsv(img);
    trgt_window = img_hsv(rect(2):rect(2)+height,rect(1):rect(1)+width,:);
    trgt = mean(mean(trgt_window))
    hr = 0.06;
    sr = 0.16;
    vr = 0.07;
    
    %fh = figure;
    
    [xc, yc, ta] = hw5_track(img,trgt,hr,sr,vr);
    
    while true
        img = imread(str_ip);
        [x,y,a] = hw5_track(img,trgt,hr,sr,vr)
        ta
        [xpos,ypos,ang] = adjust_dist(port,xpos,ypos,ang);
        scale = abs(x - axis)/axis;
        if cur_state == FIND_OBJ
            display('state: finding object');
            if x < axis*0.95
                SetDriveWheelsCreate(port,0.05*scale,-0.05*scale);
                turn_left = true;
            elseif x > axis*1.05
                SetDriveWheelsCreate(port,-0.05*scale,0.05*scale);
                turn_left = false;
            end
            cur_state = TURN2OBJ;
        elseif cur_state == TURN2OBJ
            display('state: turning to object');
            if(x > axis*0.95 && x < axis*1.05)
                SetDriveWheelsCreate(port,0.0,0.0);
                if a < ta - err
                    SetDriveWheelsCreate(port,0.05,0.05);
                    cur_state = MOVE_FWD;
                elseif a > ta + err
                    SetDriveWheelsCreate(port,-0.05,-0.05);
                    cur_state = MOVE_FWD;
                else
                    cur_state = FIND_OBJ;
                end
            else
                if((turn_left && x > axis*1.05) ||...
                   (~turn_left && x < axis*0.95))
                    cur_state = FIND_OBJ;
                end
            end
        elseif cur_state == MOVE_FWD
            display('state: moving forward');
            if ~(x > axis*0.95 && x < axis*1.05)
                cur_state = FIND_OBJ;
            elseif a > (ta - err) && a < (ta + err)
                SetDriveWheelsCreate(port,0.0,0.0);
                cur_state = FIND_OBJ;
            end
        end
        pause(0.03);    
    end
end

function [x_cen,y_cen,A] = hw5_track(img_rgb,trgt,hr,sr,vr)   
        %Reads current image
        img = rgb2hsv(img_rgb);
        %subplot(2, 3, 4);
        %imshow(img_rgb); title('RGB');
        %subplot(2, 3, 6);
        %imshow(img); title('HSV');
        %Sets up and constructs mask
        h_mask_L = img(:, :, 1) > (trgt(1) - hr);
        h_mask_H = img(:, :, 1) < (trgt(1) + hr);
        s_mask_L = img(:, :, 2) > (trgt(2) - sr);
        s_mask_H = img(:, :, 2) < (trgt(2) + sr);
        v_mask_L = img(:, :, 3) > (trgt(3) - vr);
        v_mask_H = img(:, :, 3) < (trgt(3) + vr);
        avg_mask = (h_mask_L == h_mask_H) &...
                   (s_mask_L & s_mask_H);% &...
                   %(v_mask_L == v_mask_H);
        %subplot(2, 3, 1);
        %imshow((h_mask_L == h_mask_H)); title('Hue');
        %subplot(2, 3, 2);
        %imshow((s_mask_L & s_mask_H)); title('Saturation');
        %subplot(2, 3, 3);
        %imshow((v_mask_L & v_mask_H)); title('Value');
        
    
        %Performs 15 dilations and erosions. Fills in holes in blobs that
        %are meant to connect together
        k = 10;
        processed_img = bwmorph(avg_mask, 'dilate', k);
        processed_img = bwmorph(processed_img, 'erode', k);

        %Performs 15 erosions and 15 dilations. Removes some random noise in
        %the image
        processed_img = bwmorph(processed_img, 'erode', k);
        processed_img = bwmorph(processed_img, 'dilate', k);
        
        in_mask = processed_img;
        
        %Creates a labeled image. Each label (int value) refers to an
        %object in the mask where 1's are connected
        lbl_mask = bwlabel(in_mask);
        
        %Finds the blob with the biggest area. This will be our target.
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
            
        %Calculates the centroid of the main target
        A = sum(sum(mask));
        x = 0;
        y = 0;
        [m_mask, n_mask] = size(mask);
        
        for i = 1:m_mask
            for j = 1:n_mask
                if(mask(i, j))
                    x = x + j;
                    y = y + i;
                end
            end
        end

        %Centroids
        x_cen = floor(x/A);
        y_cen = floor(y/A);

        %subplot(2,3,5);
        %imshow(mask); title('Mask');
        %hold on;
        %Draws square over centroid.
        %plot([x_cen-10;x_cen+10], [y_cen+10;y_cen+10],...
        %    'r-', 'MarkerFaceColor', [1 0 0]);
        %plot([x_cen-10;x_cen+10], [y_cen-10;y_cen-10],...
        %    'r-', 'MarkerFaceColor', [1 0 0]);
        %plot([x_cen+10;x_cen+10], [y_cen-10;y_cen+10],...
        %    'r-', 'MarkerFaceColor', [1 0 0]);
        %plot([x_cen-10;x_cen-10], [y_cen-10;y_cen+10],...
        %    'r-', 'MarkerFaceColor', [1 0 0]);
        %pause(0.05);
        %hold off;
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
