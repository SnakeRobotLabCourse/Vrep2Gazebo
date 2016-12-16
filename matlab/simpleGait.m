t = timer('TimerFcn',{@rolling, snake, snakeJoints, 1.0},'Period', 1.0, 'ExecutionMode', 'FixedRate');
start(t);

pause(1.9);

stop(t);
delete(t);
clear rolling