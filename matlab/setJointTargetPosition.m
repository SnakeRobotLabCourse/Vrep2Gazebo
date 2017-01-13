function [] = setJointTargetPosition(joint, position)
%   Summary
%   Description
    global snakeJoints;
    persistent anglePubs;
    persistent angleMsgs;
    
    if isempty(anglePubs)
        anglePubs = cell(size(snakeJoints));
        angleMsgs = cell(size(snakeJoints));
    end
    if isempty(anglePubs{joint+1})
        anglePubs{joint+1} = rospublisher(strcat('/robosnake/joint', num2str(joint), '_position_controller/command'), 'std_msgs/Float64');
        angleMsgs{joint+1} = rosmessage('std_msgs/Float64');
        fprintf('Initializing Joint %i\n', joint);
    end
    
    angleMsgs{joint+1}.Data = position;
    
    try
        send(anglePubs{joint+1}, angleMsgs{joint+1});
    catch
        % Error probably means the publisher is outdated => unset
        anglePubs{joint+1} = [];
    end
end