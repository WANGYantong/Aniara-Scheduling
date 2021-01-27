function [rate_table,max_rate,Random_time,seed_xyz] = TaskMultiRandom(set,para)

ITERATION=1e3;

candidate_table=cell(ITERATION,1);
candidate_rate=zeros(ITERATION,1);
candidate_seed=cell(ITERATION,1);
time_coll=zeros(ITERATION,1);

for ii=1:ITERATION
    [candidate_table{ii},candidate_rate(ii),time_coll(ii),candidate_seed{ii}]=...
        TaskRandom(set,para);
end
Random_time=sum(time_coll,'all');

[max_rate,index]=min(candidate_rate);
rate_table=candidate_table{index};
seed_xyz=candidate_seed{index};

end

