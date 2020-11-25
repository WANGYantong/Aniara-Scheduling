function [set,para] = TaskProduction(NUM_samples)

rng(1);

NUM_ROBOTS=10; % the number of robots
NUM_TASKS=3; % the number of tasks in each robot
T=15; % the number of time slots in each period

para=cell(NUM_samples,1);
for ii=1:NUM_samples
    para{ii}.D=randi([1,3],NUM_ROBOTS,NUM_TASKS); % for task processing duration
    para{ii}.R=randi([1,4],NUM_ROBOTS,NUM_TASKS); % for task communication data rate
    para{ii}.G_min=ones(NUM_ROBOTS,NUM_TASKS); % for minimum gap between adjacent tasks
    para{ii}.G_max=randi([3,5],NUM_ROBOTS,NUM_TASKS); % for maximum gap between adjacent tasks
end

set.NUM_ROBOTS=NUM_ROBOTS;
set.NUM_TASKS=NUM_TASKS;
set.T=T;

end

