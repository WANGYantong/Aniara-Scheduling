clear
clc


%% Generate Robots and Tasks
NUM_samples=100; % generate 100 samples
[set,para]=TaskProduction(NUM_samples); 


%% The Random Solution
random_table=cell(NUM_samples,1);
random_max=zeros(NUM_samples,1);
random_time=zeros(NUM_samples,1);
random_seed=cell(NUM_samples,1);
for ii=1:NUM_samples
    [random_table{ii},random_max(ii),random_time(ii),random_seed{ii}]=...
        TaskRondom(set,para{ii});
end


%% The Optimal Solution
optimal_table=cell(NUM_samples,1);
optimal_max=zeros(NUM_samples,1);
optimal_time=zeros(NUM_samples,1);
for ii=1:NUM_samples
    [optimal_table{ii},optimal_max(ii),optimal_time(ii)]=...
        ILP(set,para{ii},random_seed{ii});
    fprintf("finish task %d \n",ii);
end


%% Save data
filename="data/dataset"+set.NUM_ROBOTS+"_"+set.NUM_TASKS+".mat";
save(filename,'optimal_max','optimal_table','optimal_time','random_max',...
    'random_table','random_time');