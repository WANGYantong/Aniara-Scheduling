function [rate_table,max_rate,ranCas_time] = TaskRanCascade(set,para,seed_xyz)

%% Load Parameters
NUM_ROBOTS=set.NUM_ROBOTS; % the number of robots
NUM_TASKS=set.NUM_TASKS; % the number of tasks in each robot
T=set.T; % the number of time slots in each period

D=para.D; % task processing duration
R=para.R; % task communication data rate
G_min=para.G_min; % minimum gap between adjacent tasks
G_max=para.G_max; % maximum gap between adjacent tasks

solution_X=seed_xyz(1:NUM_ROBOTS*NUM_TASKS*T);
solution_X=reshape(solution_X,NUM_ROBOTS,NUM_TASKS,T);
solution_Y=seed_xyz(NUM_ROBOTS*NUM_TASKS*T+1:end-1);
solution_Y=reshape(solution_Y,NUM_ROBOTS,NUM_TASKS,T);

%% Allocate Time Slot
Y=zeros(NUM_ROBOTS,NUM_TASKS,T);
ranCas_time=0;

ST=zeros(NUM_ROBOTS,1);
for ii=1:NUM_ROBOTS
    ST(ii)=find(solution_X(ii,1,:)==1,1);
end

Y(:,1,:)=solution_Y(:,1,:);

if NUM_TASKS>=2
    for jj=2:NUM_TASKS
        % Update ES & LS according to previous allocation
        [ES,LS]=BoundUpdate(ST,G_min,G_max,D,T,jj-1);      
        
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
        ranCas_time=ranCas_time+ReducedILP_time;
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
