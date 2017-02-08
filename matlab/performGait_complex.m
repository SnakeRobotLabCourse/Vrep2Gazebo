%% Search Topics:
l=rostopic('list');
a=strfind(l,'position_controller/command');
n=[];
for k=1:length(a)
    if(~isempty(a{k}))
        n=[n,k];
    end
end
%% service creation
Join=cell(1,length(n));
JMsg=cell(1,length(n));
for k=1:length(n)
    Join{k}=rospublisher(l{n(k)}, 'std_msgs/Float64');
    JMsg{k}=rosmessage(Join{k});
end
%% sending the commands
for k=1:n
    if mod(k,2)
        oddA=C_o+A_o * ((k/n) * 0.9+0.1) * sin(w_o * t(i)+(k-1) * d_o);
        JMsg{k}.Data=oddA;
        send(Join{k},JMsg{k});
    else
        evenA=C_e+A_e * ((k/n) * 0.9+0.1) * cos(w_e * t(i)+(k-1) * d_e+d0);
        JMsg{k}.Data=evenA;
        send(Join{k},JMsg{k});
    end
end