% This is a toy model script for demonstration

clear
clc


%% Generate Robots and Tasks
rng(1);
NUM_ROBOTS=4; % the number of robots
NUM_TASKS=3; % the number of tasks in each robot
T=8; % the number of time slots in each period

D=randi([1,2],NUM_ROBOTS,NUM_TASKS); % for task processing duration
R=randi([1,4],NUM_ROBOTS,NUM_TASKS); % for task communication data rate
G_min=ones(NUM_ROBOTS,NUM_TASKS); % for minimum gap between adjacent tasks
G_max=randi([3,5],NUM_ROBOTS,NUM_TASKS); % for maximum gap between adjacent tasks

%% Generate Objective and Constraint Matrices and Vectors
NUM_DV=NUM_ROBOTS*NUM_TASKS*T*2; % the number of decision variables

% objective vector for d.v. y
obj=repmat(R,[1 1 T]);
obj=obj(:);

% mutli-objective functions
leap=NUM_ROBOTS*NUM_TASKS;
xend_point=leap*T;
str="@(x)[";
for ii=1:T
    str=str+"x("+(xend_point+leap*(ii-1)+1)+":"+(xend_point+leap*(ii))+")"+...
        "*obj("+(leap*(ii-1)+1)+":"+leap*(ii)+")"+";";
end
str=str+"]";
fun=str2func(str);

% inequality constraints



% equality constraints



% bound constraints
lb=zeros(NUM_DV,1);
ub=ones(NUM_DV,1);
