t = timer('TimerFcn',{@rolling, snake, snakeJoints, 1},'Period', 1, 'ExecutionMode', 'FixedRate');
start(t);

pause(1.9);

stop(t);
delete(t);
clear rolling