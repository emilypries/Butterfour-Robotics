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

    err = 0.5;
    timeout = 30;
    
    x_pos = 0;
    y_pos = 0;
    r_ang = 0;
    t = 0;
    tic;
    dir = [0;1];
    
    
    RANDOM_DIR = 0;
    RANDOM_ROTATE = 1;
    RANDOM_MOVE = 2;
    FIND_OBJECT = 3;
    MARK_OBJECT = 4;
    RETURN_ORIG = 5;
    
    
    m = 10;
    n = 10;
    cells = zeros(10);
    
    curr_state = RANDOM_DIR;
    
    while(true)
        
        if(curr_state == RANDOM_DIR)
            dir = floor(rand(2, 1) * 3) - 1;
            
            
            
        end
        
        
        
        
    end



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
    x = a+(d*cos(rad)) % values will be displayed
    y = b+(d*sin(rad))
end
