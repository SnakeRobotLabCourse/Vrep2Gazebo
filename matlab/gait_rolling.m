function [] = gait_rolling(obj, event) %#ok<INUSL>
%ROLLING Summary of this function goes here
%   Detailed explanation goes here

    %#ok<*TLEV>
    
    global snake;
    global snakeJoints;
    
    %initialization--------------------------------------------------------
    persistent is_initialized;
    persistent joints_h;
    persistent joints_v;
    persistent speed;
    persistent amplitude_h;
    persistent amplitude_v;
    persistent phase_v;
    persistent phase_h;
    persistent A_H;
    persistent A_V; 
    persistent P_V;
    persistent P_H;
    persistent s;
    persistent t_0;
    persistent clock;
    if isempty(is_initialized)
        is_initialized = 1;    
    
        for i = 1:5
            joints_v{i} = snakeJoints{2*i-1};
            joints_h{i} = snakeJoints{2*i};
        end
   
        %-- 2 sets of control Parameters:
        speed={-4, -4};%-2,-2
        amplitude_h = {20, 30};
        amplitude_v = {20, 30};
        phase_v = {0, 0};
        phase_h = {0, 0};

        %--Inputs for the function control (in radians) (also 2 sets)
        A_H = {amplitude_h{1} * pi / 180, amplitude_h{2} * pi / 180};
        A_V = {amplitude_v{1} * pi / 180, amplitude_v{2} * pi / 180};
        P_V = {phase_v{1} * pi / 180, phase_v{2} * pi / 180};
        P_H = {phase_h{1} * pi / 180, phase_h{2} * pi / 180};
        
        clock = rossubscriber('/clock');
        
        s = 0;
        data = receive(clock);
        t_0 = double(data(1).Clock_.Sec) + double(data(1).Clock_.Nsec)/1000000000;
    end

    %acutal work-----------------------------------------------------------
    data = receive(clock);
    t = double(data(1).Clock_.Sec) + double(data(1).Clock_.Nsec)/1000000000 - t_0;
    
    if (t > 20)  %-- Here we do a transition from one movement type to the other movement type (between time=10 and time=11):
        s = t-20;
        if (s > 1)
            s = 1;
        end
    end
    
    for i = 1:5
        setJointTargetPosition(str2double(joints_v{i}(7)), (A_V{1} * (1 - s) + A_V{2} * s) * sin(t * (speed{1} * (1 - s) + speed{2} * s) + i * (P_V{1} * (1 - s) + P_V{2} * s)));
        setJointTargetPosition(str2double(joints_h{i}(7)), (A_H{1} * (1 - s) + A_H{2} * s) * cos(t * (speed{1} * (1 - s) + speed{2} * s) + i * (P_H{1} * (1 - s) + P_H{2} * s)));
    end
end

