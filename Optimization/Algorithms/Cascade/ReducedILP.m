function [ST,solution_Y,ReducedILP_time] = ReducedILP(NUM_ROBOTS,...
    T,D,ES,LS,R,Y,index)

%% Generate Objective
NUM_DV=NUM_ROBOTS*T*2+1;

% objective vector 
obj=sparse([zeros(1,NUM_DV-1),1]);


%% Inequality Constraints
D=D(:,index);
Aineq=spalloc(NUM_ROBOTS*2+sum(D,'all')*T+T,NUM_DV,...
    T*NUM_ROBOTS*2+2*sum(D,'all')*T+(NUM_ROBOTS+1)*T);
bineq=zeros(NUM_ROBOTS*2+sum(D,'all')*T+T,1);

clearer1=zeros(NUM_ROBOTS,T);
clearer11=clearer1(:);
counter=1;

% ES constraints
for ii=1:NUM_ROBOTS
    xtemp=clearer1;
    for kk=1:T
        xtemp(ii,kk)=-kk;
    end
    xtemp=sparse([xtemp(:);clearer11;0]);
    Aineq(counter,:)=xtemp';
    bineq(counter)=-ES(ii);
    counter=counter+1;
end

% LS constraints
for ii=1:NUM_ROBOTS
    xtemp=clearer1;
    for kk=1:T
        xtemp(ii,kk)=kk;
    end
    xtemp=sparse([xtemp(:);clearer11;0]);
    Aineq(counter,:)=xtemp';
    bineq(counter)=LS(ii);
    counter=counter+1;
end

% the association between x & y
for ii=1:NUM_ROBOTS
    for kk=1:T
        for ll=1:D(ii)
            xtemp=clearer1;
            ytemp=clearer1;
            xtemp(ii,kk)=1;
            if (kk+ll-1) <= T
                ytemp(ii,kk+ll-1)=-1;
            else
                ytemp(ii,T)=-1;
            end
            xtemp=sparse([xtemp(:);ytemp(:);0]);
            Aineq(counter,:)=xtemp';
            counter=counter+1;
        end
    end
end

% minimax conversion
for kk=1:T
    ytemp=clearer1;
    for ii=1:NUM_ROBOTS
        ytemp(ii,kk)=R(ii,index);
    end
    ytemp=sparse([clearer11;ytemp(:);-1]);
    Aineq(counter,:)=ytemp';
    bineq(counter)=-sum(R.*Y(:,:,kk),'all');
    counter=counter+1;
end


%% Equality Constraints
Aeq=spalloc(NUM_ROBOTS*2,NUM_DV,T*NUM_ROBOTS*2);
beq=ones(NUM_ROBOTS*2,1);

counter=1;
% only one starting time for each task
for ii=1:NUM_ROBOTS
    xtemp=clearer1;
    xtemp(ii,:)=1;
    xtemp=sparse([xtemp(:);clearer11;0]);
    Aeq(counter,:)=xtemp';
    counter=counter+1;
end

% the occupying slot is equal with task processing duration 
for ii=1:NUM_ROBOTS
    ytemp=clearer1;
    ytemp(ii,:)=1;
    ytemp=sparse([clearer11;ytemp(:);0]);
    Aeq(counter,:)=ytemp';
    beq(counter)=D(ii);
    counter=counter+1;
end


%% Bound Constraints
lb=zeros(1,NUM_DV);
ub=ones(1,NUM_DV);
ub(end)=inf;


%% Solve the Problem
opts=optimoptions('intlinprog','Display','off','MaxTime',28800);
intcon=1:NUM_DV;

RobotSchedule.f=obj;
RobotSchedule.intcon=intcon; % integer point
RobotSchedule.Aineq=Aineq;
RobotSchedule.bineq=bineq;
RobotSchedule.Aeq=Aeq;
RobotSchedule.beq=beq;
RobotSchedule.lb=lb;
RobotSchedule.ub=ub;
RobotSchedule.solver='intlinprog';
RobotSchedule.options=opts;

tic;
solution=intlinprog(RobotSchedule);
ReducedILP_time=toc;

if isempty(solution)
    ST=zeros(NUM_ROBOTS,1);
    solution_Y=zeros(NUM_ROBOTS,T);
    return
end

%% Reshape the Solution
solution=round(solution);
solution_X=solution(1:NUM_ROBOTS*T);
solution_X=reshape(solution_X,NUM_ROBOTS,T);
ST=zeros(NUM_ROBOTS,1);
for ii=1:NUM_ROBOTS
    ST(ii)=find(solution_X(ii,:)==1);
end

solution_Y=solution(NUM_ROBOTS*T+1:end-1);
solution_Y=reshape(solution_Y,NUM_ROBOTS,T);


end

