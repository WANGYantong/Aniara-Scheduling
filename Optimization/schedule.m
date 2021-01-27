clear
clc

addpath(genpath(pwd));
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
        TaskRandom(set,para{ii});
end


%% The MultiRandom Solution
multiR_table=cell(NUM_samples,1);
multiR_max=zeros(NUM_samples,1);
multiR_time=zeros(NUM_samples,1);
multiR_seed=cell(NUM_samples,1);
for ii=1:NUM_samples
    [multiR_table{ii},multiR_max(ii),multiR_time(ii),multiR_seed{ii}]=...
        TaskMultiRandom(set,para{ii});
    fprintf("MultiRandom finish task %d \n",ii);
end


%% The Cascade Solution
cas_table=cell(NUM_samples,1);
cas_max=zeros(NUM_samples,1);
cas_time=zeros(NUM_samples,1);
parfor ii=1:NUM_samples
    [cas_table{ii},cas_max(ii),cas_time(ii)]=...
        TaskCascade(set,para{ii},random_seed{ii});
    fprintf("Cascade finish task %d \n",ii);
end


%% The RandomCascade Solution
ranCas_table=cell(NUM_samples,1);
ranCas_max=zeros(NUM_samples,1);
ranCas_time=zeros(NUM_samples,1);
parfor ii=1:NUM_samples
    [ranCas_table{ii},ranCas_max(ii),ranCas_time(ii)]=...
        TaskRanCascade(set,para{ii},multiR_seed{ii});
    fprintf("Random+Cascade finish task %d \n",ii);
end


%% The Optimal Solution
optimal_table=cell(NUM_samples,1);
optimal_max=zeros(NUM_samples,1);
optimal_time=zeros(NUM_samples,1);
parfor ii=1:NUM_samples
    [optimal_table{ii},optimal_max(ii),optimal_time(ii)]=...
        ILP(set,para{ii},multiR_seed{ii});
    fprintf("ILP finish task %d \n",ii);
end


%% Save data
filename="data_2021/dataset"+set.NUM_ROBOTS+"_"+set.NUM_TASKS+".mat";
save(filename,'optimal_max','optimal_table','optimal_time',...
    'random_max','random_table','random_time',...
    'cas_max','cas_table','cas_time',...
    'multiR_max','multiR_table','multiR_time',...
    'ranCas_max','ranCas_table','ranCas_time');