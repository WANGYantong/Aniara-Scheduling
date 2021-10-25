% This is a toy model script for demonstration

clear
clc


%% Generate Robots and Tasks
rng(1);
NUM_ROBOTS=5; % the number of robots
NUM_TASKS=3; % the number of tasks in each robot
T=8; % the number of time slots in each period

D=randi([1,2],NUM_ROBOTS,NUM_TASKS); % for task processing duration
R=randi([1,4],NUM_ROBOTS,NUM_TASKS); % for task communication data rate
G_min=ones(NUM_ROBOTS,NUM_TASKS); % for minimum gap between adjacent tasks
G_max=randi([3,5],NUM_ROBOTS,NUM_TASKS); % for maximum gap between adjacent tasks


%% Generate Objective 
NUM_DV=NUM_ROBOTS*NUM_TASKS*T*2+1; % the number of decision variables (d.v.)

% objective vector for d.v. z
obj=sparse([zeros(1,NUM_DV-1),1]);

%% Generate Constraints
%% inequality constraints
Aineq=spalloc(...
    NUM_ROBOTS*(NUM_TASKS-1)+NUM_ROBOTS*(NUM_TASKS-1)+NUM_ROBOTS+sum(D,'all')*T+T,...
    NUM_DV,2*T*NUM_ROBOTS*(NUM_TASKS-1)+2*T*NUM_ROBOTS*(NUM_TASKS-1)+...
    T*NUM_ROBOTS+2*sum(D,'all')*T+(NUM_ROBOTS*NUM_TASKS+1)*T);
bineq=zeros(NUM_ROBOTS*(NUM_TASKS-1)+NUM_ROBOTS*(NUM_TASKS-1)+NUM_ROBOTS+...
    sum(D,'all')*T+T,1);

clearer1=zeros(NUM_ROBOTS,NUM_TASKS,T);
clearer11=clearer1(:);

% minimum gap constraints
counter=1;
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS-1
        xtemp=clearer1;
        for kk=1:T
            xtemp(ii,jj,kk)=kk+D(ii,jj);
            xtemp(ii,jj+1,kk)=-kk;
        end
        xtemp=sparse([xtemp(:);clearer11;0]);
        Aineq(counter,:)=xtemp';
        bineq(counter)=-G_min(ii,jj);
        counter=counter+1;
    end
end            
            
% maximum gap constraints
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS-1
        xtemp=clearer1;
        for kk=1:T
            xtemp(ii,jj,kk)=-(kk+D(ii,jj));
            xtemp(ii,jj+1,kk)=kk;
        end
        xtemp=sparse([xtemp(:);clearer11;0]);
        Aineq(counter,:)=xtemp';
        bineq(counter)=G_max(ii,jj);
        counter=counter+1;
    end
end      

% the last task constraints
for ii=1:NUM_ROBOTS
    xtemp=clearer1;
    for kk=1:T
        xtemp(ii,NUM_TASKS,kk)=kk+D(ii,NUM_TASKS)-1;
    end
    xtemp=sparse([xtemp(:);clearer11;0]);
    Aineq(counter,:)=xtemp';
    bineq(counter,:)=T-G_min(ii,NUM_TASKS);
    counter=counter+1;
end

% the association between d.v. x and y
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        for kk=1:T
            for ll=1:D(ii,jj)
                xtemp=clearer1;
                ytemp=clearer1;
                xtemp(ii,jj,kk)=1;
                if (kk+ll-1) <= T
                    ytemp(ii,jj,kk+ll-1)=-1;
                else
                    ytemp(ii,jj,T)=-1;
                end
                xtemp=sparse([xtemp(:);ytemp(:);0]);
                Aineq(counter,:)=xtemp';
                counter=counter+1;
            end
        end
    end
end

% minimax conversion
for kk=1:T
    ytemp=clearer1;
    for ii=1:NUM_ROBOTS
        for jj=1:NUM_TASKS
            ytemp(ii,jj,kk)=R(ii,jj);
        end
    end
    ytemp=sparse([clearer11;ytemp(:);-1]);
    Aineq(counter,:)=ytemp';
    counter=counter+1;
end


%% equality constraints
Aeq=spalloc(NUM_ROBOTS*NUM_TASKS+NUM_ROBOTS*NUM_TASKS,...
    NUM_DV,NUM_ROBOTS*NUM_TASKS*T+NUM_ROBOTS*NUM_TASKS*T);
beq=ones(NUM_ROBOTS*NUM_TASKS,1);

counter=1;
% only one starting time for each task
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        xtemp=clearer1;
        xtemp(ii,jj,:)=1;
        xtemp=sparse([xtemp(:);clearer11;0]);
        Aeq(counter,:)=xtemp';
        counter=counter+1;
    end
end

% the occupying slot is equal with task processing duration 
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        ytemp=clearer1;
        ytemp(ii,jj,:)=1;
        ytemp=sparse([clearer11;ytemp(:);0]);
        Aeq(counter,:)=ytemp';
        beq(counter)=D(ii,jj);
        counter=counter+1;
    end
end


%% bound constraints
lb=zeros(1,NUM_DV);
ub=ones(1,NUM_DV);
ub(end)=inf;

%% solve the problem
opts=optimoptions('intlinprog','Display','off','MaxTime',3600);
intcon=1:NUM_DV;

RobotSchedule.f=obj;
RobotSchedule.intcon=intcon; % initial point
RobotSchedule.Aineq=Aineq;
RobotSchedule.bineq=bineq;
RobotSchedule.Aeq=Aeq;
RobotSchedule.beq=beq;
RobotSchedule.lb=lb;
RobotSchedule.ub=ub;
RobotSchedule.solver='intlinprog';
RobotSchedule.options=opts;

[solution,fval,exitflag,output]=intlinprog(RobotSchedule);

if isempty(solution)
    disp('intlinprog did not return a solution.')
    return
end


%% reshape the solution
solution=round(solution);
solution_X=solution(1:NUM_ROBOTS*NUM_TASKS*T);
solution_X=reshape(solution_X,NUM_ROBOTS,NUM_TASKS,T);
solution_Y=solution(NUM_ROBOTS*NUM_TASKS*T+1:end-1);
solution_Y=reshape(solution_Y,NUM_ROBOTS,NUM_TASKS,T);
solution_Z=solution(end);


%% rate table
rate_table=zeros(NUM_ROBOTS+1,T);
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        rate_table(ii,:)=rate_table(ii,:)+...
            transpose(squeeze(solution_Y(ii,jj,:)))*R(ii,jj);
    end
end
rate_table(end,:)=sum(rate_table,1);