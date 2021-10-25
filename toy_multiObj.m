% This is a toy model script for demonstration

clear
clc


%% Generate Robots and Tasks
rng(1);
NUM_ROBOTS=3; % the number of robots
NUM_TASKS=3; % the number of tasks in each robot
T=8; % the number of time slots in each period

D=randi([1,2],NUM_ROBOTS,NUM_TASKS); % for task processing duration
R=randi([1,4],NUM_ROBOTS,NUM_TASKS); % for task communication data rate
G_min=ones(NUM_ROBOTS,NUM_TASKS); % for minimum gap between adjacent tasks
G_max=randi([3,5],NUM_ROBOTS,NUM_TASKS); % for maximum gap between adjacent tasks


%% Generate Objective 
NUM_DV=NUM_ROBOTS*NUM_TASKS*T*2; % the number of decision variables (d.v.)

% objective vector for d.v. y
obj=R(:);

% mutli-objective functions
leap=NUM_ROBOTS*NUM_TASKS;
xend_point=leap*T;
str="@(x)[";
for ii=1:T
    str=str+"x("+(xend_point+leap*(ii-1)+1)+":"+(xend_point+leap*(ii))+...
        ")*obj;";
end
str=str+"]";
fun=eval(str); % fun=str2func(str) not work here because Function handles...
               % created using str2func do not have access to variables...
               % outside of their local workspace or to nested functions. 


%% Generate Constraints
%% inequality constraints
Aineq=spalloc(...
    NUM_ROBOTS*(NUM_TASKS-1)+NUM_ROBOTS*(NUM_TASKS-1)+NUM_ROBOTS+sum(D,'all')*T,...
    NUM_DV,2*T*NUM_ROBOTS*(NUM_TASKS-1)+2*T*NUM_ROBOTS*(NUM_TASKS-1)+...
    T*NUM_ROBOTS+2*sum(D,'all')*T);
bineq=zeros(NUM_ROBOTS*(NUM_TASKS-1)+NUM_ROBOTS*(NUM_TASKS-1)+NUM_ROBOTS+...
    sum(D,'all')*T,1);

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
        xtemp=sparse([xtemp(:);clearer11]);
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
        xtemp=sparse([xtemp(:);clearer11]);
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
    xtemp=sparse([xtemp(:);clearer11]);
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
                xtemp=sparse([xtemp(:);ytemp(:)]);
                Aineq(counter,:)=xtemp';
                counter=counter+1;
            end
        end
    end
end

%% equality constraints
Aeq=spalloc(NUM_ROBOTS*NUM_TASKS,NUM_DV,NUM_ROBOTS*NUM_TASKS*T);
beq=ones(NUM_ROBOTS*NUM_TASKS,1);

counter=1;
% only one starting time for each task
for ii=1:NUM_ROBOTS
    for jj=1:NUM_TASKS
        xtemp=clearer1;
        xtemp(ii,jj,:)=1;
        xtemp=sparse([xtemp(:);clearer11]);
        Aeq(counter,:)=xtemp';
        counter=counter+1;
    end
end


%% bound constraints
lb=zeros(1,NUM_DV);
ub=ones(1,NUM_DV);


%% solve the problem
opts=optimoptions('fminimax','Display','off','PlotFcn','optimplotfval',...
    'FunValCheck','on');

RobotSchedule.objective=fun;
RobotSchedule.x0=ones(1,NUM_DV); % initial point
RobotSchedule.Aineq=Aineq;
RobotSchedule.bineq=bineq;
RobotSchedule.Aeq=Aeq;
RobotSchedule.beq=beq;
RobotSchedule.lb=lb;
RobotSchedule.ub=ub;
RobotSchedule.solver='fminimax';
RobotSchedule.options=opts;

[solution,fval,maxfval,exitflag,output]=fminimax(RobotSchedule);

if isempty(solution)
    disp('intlinprog did not return a solution.')
    return
end


%% reshape the solution
% solution=round(solution);
solution_X=solution(1:xend_point);
solution_X=reshape(solution_X,NUM_ROBOTS,NUM_TASKS,T);
solution_Y=solution(xend_point+1:end);
solution_Y=reshape(solution_Y,NUM_ROBOTS,NUM_TASKS,T);