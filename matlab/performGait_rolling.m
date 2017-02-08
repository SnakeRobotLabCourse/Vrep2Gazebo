t = timer('TimerFcn',{@gait_rolling},'Period', 0.2, 'ExecutionMode', 'FixedRate');
start(t);

pause(20);

stop(t);
delete(t);
resetSnake;
clear rolling