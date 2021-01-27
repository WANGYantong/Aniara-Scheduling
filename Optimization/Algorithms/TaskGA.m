function [rate_table,max_rate,ga_time] = TaskGA(set,para,seed_xyz)

if nargin<3
    seed_xyz=[];
end

%% load Parameters
NUM_ROBOTS=set.NUM_ROBOTS; % the number of robots
NUM_TASKS=set.NUM_TASKS; % the number of tasks in each robot
T=set.T; % the number of time slots in each period

D=para.D; % task processing duration
R=para.R; % task communication data rate
G_min=para.G_min; % minimum gap between adjacent tasks
G_max=para.G_max; % maximum gap between adjacent tasks


%% Generate Objective
NUM_DV=NUM_ROBOTS*NUM_TASKS*T*2+1; % the number of decision variables (d.v.)

% objective vector for d.v. z
% obj=sparse([zeros(1,NUM_DV-1),1]);

fun=@(x)(x(end));


%% Inequality Constraints
% linear equality constraints are not supported yet
% therefore these equalities are converted to \leq and \gea constraints
Aineq=spalloc(NUM_ROBOTS*(NUM_TASKS-1)*2+NUM_ROBOTS+sum(D,'all')*T...
    +T+NUM_ROBOTS*NUM_TASKS*4,...
    NUM_DV,...
    2*T*NUM_ROBOTS*(NUM_TASKS-1)*2+T*NUM_ROBOTS+2*sum(D,'all')*T...
    +(NUM_ROBOTS*NUM_TASKS+1)*T+T*NUM_ROBOTS*NUM_TASKS*4);
bineq=zeros(NUM_ROBOTS*(NUM_TASKS-1)*2+NUM_ROBOTS+sum(D,'all')*T...
    +T+NUM_ROBOTS*NUM_TASKS*4,1);

clearer1=zeros(NUM_ROBOTS,NUM_TASKS,T);
clearer11=clearer1(:);
counter=1;

% minimum gap constraints
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
    bineq(counter)=T-G_min(ii,NUM_TASKS);
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

% equality conversion: only one starting time for each task
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        xtemp=clearer1;
        xtemp(ii,jj,:)=1;
        xtemp=sparse([xtemp(:);clearer11;0]);
        Aineq(counter,:)=xtemp';
        bineq(counter)=1;
        counter=counter+1;
    end
end

for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        xtemp=clearer1;
        xtemp(ii,jj,:)=-1;
        xtemp=sparse([xtemp(:);clearer11;0]);
        Aineq(counter,:)=xtemp';
        bineq(counter)=-1;
        counter=counter+1;
    end
end

% equality conversion: the occupying slot is equal with task processing
% duration
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        ytemp=clearer1;
        ytemp(ii,jj,:)=1;
        ytemp=sparse([clearer11;ytemp(:);0]);
        Aineq(counter,:)=ytemp';
        bineq(counter)=D(ii,jj);
        counter=counter+1;
    end
end

for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        ytemp=clearer1;
        ytemp(ii,jj,:)=-1;
        ytemp=sparse([clearer11;ytemp(:);0]);
        Aineq(counter,:)=ytemp';
        bineq(counter)=-D(ii,jj);
        counter=counter+1;
    end
end


%% Bound Constraints
lb=zeros(1,NUM_DV);
ub=ones(1,NUM_DV);
ub(end)=inf;


%% Solve the Problem
opts=optimoptions(@ga,...
                  'PopulationSize',1e3,...
                  'MaxGenerations',1e2,...
                  'EliteCount',1e2,...
                  'FunctionTolerance',1e-8,...
                  'CrossoverFraction',0.9,...
                  'InitialPopulationMatrix',seed_xyz');

RobotGA.fitnessfcn=fun;
RobotGA.nvars=NUM_DV;
RobotGA.Aineq=Aineq;
RobotGA.Bineq=bineq;
RobotGA.Aeq=[];
RobotGA.Beq=[];
RobotGA.lb=lb;
RobotGA.ub=ub;
RobotGA.nonlcon=[];
RobotGA.IntCon=1:NUM_DV;
RobotGA.solver='ga';
RobotGA.options=opts;

tic;
solution=ga(RobotGA);
ga_time=toc;


%% Reshape the Solution
solution=round(solution);
solution_Y=solution(NUM_ROBOTS*NUM_TASKS*T+1:end-1);
solution_Y=reshape(solution_Y,NUM_ROBOTS,NUM_TASKS,T);
max_rate=solution(end);


%% Rate Table
rate_table=zeros(NUM_ROBOTS+1,T);
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        rate_table(ii,:)=rate_table(ii,:)+...
            transpose(squeeze(solution_Y(ii,jj,:)))*R(ii,jj);
    end
end
rate_table(end,:)=sum(rate_table,1);

end

