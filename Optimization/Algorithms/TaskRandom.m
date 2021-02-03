function [rate_table,max_rate,Random_time,seed_xyz] = TaskRandom(set,para)

%% Load Parameters
NUM_ROBOTS=set.NUM_ROBOTS; % the number of robots
NUM_TASKS=set.NUM_TASKS; % the number of tasks in each robot
T=set.T; % the number of time slots in each period

D=para.D; % for task processing duration
R=para.R; % for task communication data rate
G_min=para.G_min; % for minimum gap between adjacent tasks
G_max=para.G_max; % for maximum gap between adjacent tasks

    
%% Allocate Time Slot
time_loop=1;
tic;
for xx=1:time_loop  % for accurate tic-toc time counting
    
solution_Y=zeros(NUM_ROBOTS,NUM_TASKS,T);
solution_X=solution_Y;

for ii=1:NUM_ROBOTS
    ES=1; % earliest start time
    LS=T-sum(G_min(ii,:))-sum(D(ii,:))+1; % latest start time
    if ES>LS
        disp('infeasible input')
        return
    end  
    ST=randi([ES,LS]); % allocated start time
    ET=ST+D(ii,1)-1; % according end time
    solution_Y(ii,1,ST:ET)=1;
    solution_X(ii,1,ST)=1;
    if NUM_TASKS>=2
        for jj=2:NUM_TASKS
            ES=ET+G_min(ii,jj-1)+1;
            LS1=ET+G_max(ii,jj-1)+1;
            LS2=T-sum(G_min(ii,jj:end))-sum(D(ii,jj:end))+1;
            LS=min([LS1,LS2]);
            if ES>LS
                disp('infeasible input')
                return
            end
            ST=randi([ES,LS]);
            ET=ST+D(ii,jj)-1;
            solution_Y(ii,jj,ST:ET)=1;
            solution_X(ii,jj,ST)=1;
        end
    end
end

end
timer=toc;
Random_time=timer/time_loop;

%% Rate Table
rate_table=zeros(NUM_ROBOTS+1,T);
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        rate_table(ii,:)=rate_table(ii,:)+...
            transpose(squeeze(solution_Y(ii,jj,:)))*R(ii,jj);
    end
end
rate_table(end,:)=sum(rate_table,1);
max_rate=max(rate_table(end,:));

seed_xyz=[solution_X(:);solution_Y(:);max_rate];

end

