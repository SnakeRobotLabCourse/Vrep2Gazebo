function [] = rolling(obj, event, timeStep) %#ok<INUSL>
%ROLLING Summary of this function goes here
%   Detailed explanation goes here

    %#ok<*TLEV>
    
    global snake;
    global snakeJoints;
    
    %initialization--------------------------------------------------------
    persistent is_initialized;
    persistent amplitude;
    persistent speed;
    persistent joints_h;
    persistent joints_v;
    persistent t;
    if isempty(is_initialized)
        is_initialized = 1;    
    
        for i = 1:5
            joints_v{i} = snakeJoints{2*i-1};
            joints_h{i} = snakeJoints{2*i};
        end
        
        speed = 0.2;
        amplitude = 1;
        
        t = 0;
    end

    %acutal work-----------------------------------------------------------
    t = t + timeStep;
    
    for i = 1:5
        setJointTargetPosition(str2double(joints_v{i}(7)), 0.5);%amplitude * sin(t * speed));
        setJointTargetPosition(str2double(joints_h{i}(7)), 0);%amplitude * cos(t * speed));
    end
end

