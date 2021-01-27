function [rate_table,max_rate,cas_time] = TaskCascade(set,para,seed_xyz)

%% Load Parameters
NUM_ROBOTS=set.NUM_ROBOTS; % the number of robots
NUM_TASKS=set.NUM_TASKS; % the number of tasks in each robot
T=set.T; % the number of time slots in each period

D=para.D; % task processing duration
R=para.R; % task communication data rate
G_min=para.G_min; % minimum gap between adjacent tasks
G_max=para.G_max; % maximum gap between adjacent tasks


%% Allocate Time Slot
% Initialize ES & LS, the Boundary for Constraints 1
[ES,LS]=BoundInitialize(NUM_ROBOTS,G_min,D,T);

Y=zeros(NUM_ROBOTS,NUM_TASKS,T);
cas_time=0;
for jj=1:NUM_TASKS
    
    % Solve Reduced ILP Model
    [ST,solution_Y,ReducedILP_time]=ReducedILP(NUM_ROBOTS,...
        T,D,ES,LS,R,Y,jj);
    
    if ST==0
        Y=seed_xyz(NUM_ROBOTS*NUM_TASKS*T+1:end-1);
        Y=reshape(Y,NUM_ROBOTS,NUM_TASKS,T);
        break;
    end
    
    % Update Y
    Y(:,jj,:)=solution_Y;
    
    % Count Running Time
    cas_time=cas_time+ReducedILP_time;
    
    if (NUM_TASKS>=2) && (jj<NUM_TASKS)
        % Update ES & LS Accroding to ILP Allocation
        [ES,LS]=BoundUpdate(ST,G_min,G_max,D,T,jj);
    end
    
end


%% Rate Table
rate_table=zeros(NUM_ROBOTS+1,T);
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        rate_table(ii,:)=rate_table(ii,:)+...
            transpose(squeeze(Y(ii,jj,:)))*R(ii,jj);
    end
end
rate_table(end,:)=sum(rate_table,1);
max_rate=max(rate_table(end,:));

end

