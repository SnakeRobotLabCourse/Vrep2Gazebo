
% set all Joints to a default position

for i = 0:9
    %setJointTargetPosition(i, (mod(i, 2) == 0) * 0.5);
    setJointTargetPosition(i, 0);
end