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
    
% main function - roomba follows planned path trajectory
% port is robot object returned by RoombaInit
function hw4_team_04(port)
    path = fopen('path.txt');
    npts = str2double(fgetl(path));
    ppts = [];
    for i = 1:npts
        line = fgetl(path);
        sp = textscan(line,'%s');
        sp = sp{1};
        ppts(i,:) = [str2double(sp(1)) str2double(sp(2))];
    end 
    
    % spatial descriptors of roomba
    x = ppts(1,1);
    y = ppts(1,2);
    r = pi/2;
    angerr = pi/36;
    poserr = 0.175;
    % states for process
    TURN2POINT = 0;
    HEAD2POINT = 1;
    WHEN2STOPP = 2;
    
    idx = 2;
    goal_idx = size(ppts,1);
    cur_state = TURN2POINT;
    
    turn_r = 0;
    
    ppts
    
    while(true)
        [x,y,r] = adjust_dist(port,x,y,r);
        turn_r
        
        if(idx == goal_idx)
            if (x > ppts(idx,1) - poserr && x < ppts(idx, 1) + poserr &&...
                y > ppts(idx,2) - poserr && y < ppts(idx, 2) + poserr)
                break;
            end
        end
        
        if (cur_state == TURN2POINT)
            dir2turn = ppts(idx,:)-[x,y];
            ang2turn = atan2(dir2turn(2), dir2turn(1))
            turn_r = ang2turn;
            if ang2turn < 0
                SetDriveWheelsCreate(port,-0.1,0.1);
            elseif ang2turn > 0
                SetDriveWheelsCreate(port,0.1,-0.1);
            end
            cur_state = HEAD2POINT;
            
        elseif (cur_state == HEAD2POINT)
            if(r > turn_r - angerr && r < turn_r + angerr)
                SetDriveWheelsCreate(port,0.0,0.0);
                pause(0.05);
                SetDriveWheelsCreate(port,0.2,0.2); % .2 fast .2 furious
            end
            cur_state = WHEN2STOPP;
        elseif (cur_state == WHEN2STOPP)
            if (x > ppts(idx,1) - poserr && x < ppts(idx, 1) + poserr &&...
                y > ppts(idx,2) - poserr && y < ppts(idx, 2) + poserr)
                SetDriveWheelsCreate(port,0.0,0.0);
                pause(0.05);
                idx = idx + 1;
                cur_state = TURN2POINT;
            end
        end
        pause(0.05);
    end
    SetDriveWheelsCreate(port,0.0,0.0);

end

% updates the spatial descriptors
function [x, y, r] = adjust_dist(port, a, b, rad) 
    r = rad + AngleSensorRoomba(port);
    %if (r < 0)
    %    r = r + (2*pi);
    %end
    %if (r > 2*pi)
    %    r = r - (2*pi);
    %end
    d = DistanceSensorRoomba(port);
    x = a+d*cos(rad) % values will be displayed
    y = b+d*sin(rad)
    r
end