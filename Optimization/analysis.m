clear
clc


%% loading data
ROBOTS=2:2:10;
TASKS=1:3;

% Multi Random Alg Vs Random Gain
MultiR_mean_gain=zeros(length(ROBOTS),length(TASKS));
MultiR_max_gain=zeros(length(ROBOTS),length(TASKS));
MultiR_min_gain=zeros(length(ROBOTS),length(TASKS));
% Cascade Alg Vs Random Gain
Cascade_mean_gain=zeros(length(ROBOTS),length(TASKS));
Cascade_max_gain=zeros(length(ROBOTS),length(TASKS));
Cascade_min_gain=zeros(length(ROBOTS),length(TASKS));
% Random+Cascade Alg Vs Random Gain
RanCas_mean_gain=zeros(length(ROBOTS),length(TASKS));
RanCas_max_gain=zeros(length(ROBOTS),length(TASKS));
RanCas_min_gain=zeros(length(ROBOTS),length(TASKS));
% Optimal Alg Vs Random Gain
Optimal_mean_gain=zeros(length(ROBOTS),length(TASKS));
Optimal_max_gain=zeros(length(ROBOTS),length(TASKS));
Optimal_min_gain=zeros(length(ROBOTS),length(TASKS));

time_random=zeros(length(ROBOTS),length(TASKS));
time_multiR=zeros(length(ROBOTS),length(TASKS));
time_cascade=zeros(length(ROBOTS),length(TASKS));
time_rancas=zeros(length(ROBOTS),length(TASKS));
time_optimal=zeros(length(ROBOTS),length(TASKS));


for ii=1:length(ROBOTS)
    for jj=1:length(TASKS)
        filename="data_2021/dataset"+ROBOTS(ii)+"_"+TASKS(jj)+".mat";
        buff=load(filename);
        
        gain=(buff.random_max-buff.multiR_max)./buff.random_max;
        MultiR_mean_gain(ii,jj)=mean(gain);
        MultiR_max_gain(ii,jj)=max(gain);
        MultiR_min_gain(ii,jj)=min(gain);
        
        gain=(buff.random_max-buff.cas_max)./buff.random_max;
        Cascade_mean_gain(ii,jj)=mean(gain);
        Cascade_max_gain(ii,jj)=max(gain);
        Cascade_min_gain(ii,jj)=min(gain);
        
        gain=(buff.random_max-buff.ranCas_max)./buff.random_max;
        RanCas_mean_gain(ii,jj)=mean(gain);
        RanCas_max_gain(ii,jj)=max(gain);
        RanCas_min_gain(ii,jj)=min(gain);
        
        gain=(buff.random_max-buff.optimal_max)./buff.random_max;
        Optimal_mean_gain(ii,jj)=mean(gain);
        Optimal_max_gain(ii,jj)=max(gain);
        Optimal_min_gain(ii,jj)=min(gain);
        
        time_random(ii,jj)=mean(buff.random_time);
        time_multiR(ii,jj)=mean(buff.multiR_time);
        time_cascade(ii,jj)=mean(buff.cas_time);
        time_rancas(ii,jj)=mean(buff.ranCas_time+buff.multiR_time);
        time_optimal(ii,jj)=mean(buff.optimal_time);
    end
end

%% drawing figure
x=ROBOTS;
y=cell(length(TASKS),1);
fontsize1=24;
fontsize2=28;
for jj=1:length(TASKS)
    y{jj}=[MultiR_min_gain(:,jj),MultiR_mean_gain(:,jj),...
        Optimal_min_gain(:,jj),Optimal_mean_gain(:,jj)];
    y{jj}=round(y{jj},3);
    
    figure(jj);
    H=bar(x,y{jj},'grouped');
    
    H(1).FaceColor='#bbbbbb';
    xtips1=H(1).XEndPoints;
    ytips1=H(1).YEndPoints;
    labels1=string(H(1).YData);
    text(xtips1,ytips1,labels1,'HorizontalAlignment','center','FontSize',...
        fontsize1,'FontWeight','bold','VerticalAlignment','bottom');
    
    H(2).FaceColor='#999999';
    xtips2=H(2).XEndPoints;
    ytips2=H(2).YEndPoints;
    labels2=string(H(2).YData);
    text(xtips2,ytips2,labels2,'HorizontalAlignment','center','FontSize',...
        fontsize1,'FontWeight','bold','VerticalAlignment','bottom');
    
    H(3).FaceColor='#666666';
    xtips3=H(3).XEndPoints;
    ytips3=H(3).YEndPoints;
    labels3=string(H(3).YData);
    text(xtips3,ytips3,labels3,'HorizontalAlignment','center','FontSize',...
        fontsize1,'FontWeight','bold','VerticalAlignment','bottom');
    
    H(4).FaceColor='#333333';
    xtips4=H(4).XEndPoints;
    ytips4=H(4).YEndPoints;
    labels4=string(H(4).YData);
    text(xtips4,ytips4,labels4,'HorizontalAlignment','center','FontSize',...
        fontsize1,'FontWeight','bold','VerticalAlignment','bottom');
    
    xlabel('Number of Robots','FontSize',fontsize2,'FontWeight','bold');
    ylabel('Performance Gain','FontSize',fontsize2,'FontWeight','bold');
    ylim([0,0.6]);
    set(gca,'fontsize',fontsize2,'FontWeight','bold');
    lgd=legend({'min m-R gain','mean m-R gain',...
        'min optimal gain','mean optimal gain'},'location','northwest',...
        'NumColumns',2);
    lgd.FontSize=fontsize2;
    lgd.FontWeight='bold';
    
end


