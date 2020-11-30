clear
clc


%% loading data
ROBOTS=2:2:10;
TASKS=1:3;
mean_gain=zeros(length(ROBOTS),length(TASKS));
max_gain=mean_gain;
min_gain=mean_gain;
time_optimal=zeros(length(ROBOTS),length(TASKS));
time_random=time_optimal;

for ii=1:length(ROBOTS)
    for jj=1:length(TASKS)
        filename="data/dataset"+ROBOTS(ii)+"_"+TASKS(jj)+".mat";
        buff=load(filename);
        gain=(buff.random_max-buff.optimal_max)./buff.random_max;
        mean_gain(ii,jj)=mean(gain);
        max_gain(ii,jj)=max(gain);
        min_gain(ii,jj)=min(gain);
        time_optimal(ii,jj)=mean(buff.optimal_time);
        time_random(ii,jj)=mean(buff.random_time);
    end
end

%% drawing figure
x=ROBOTS;
y=cell(length(TASKS),1);
fontsize1=24;
fontsize2=28;
for jj=1:length(TASKS)
    y{jj}=[min_gain(:,jj),mean_gain(:,jj),max_gain(:,jj)];
    y{jj}=round(y{jj},3);
    
    figure(jj);
    H=bar(x,y{jj},'grouped');
    
    H(1).FaceColor='#bbbbbb';
    xtips1=H(1).XEndPoints;
    ytips1=H(1).YEndPoints;
    labels1=string(H(1).YData);
    text(xtips1,ytips1,labels1,'HorizontalAlignment','center','FontSize',...
        fontsize1,'FontWeight','bold','VerticalAlignment','bottom');
    
    H(2).FaceColor='#707070';
    xtips2=H(2).XEndPoints;
    ytips2=H(2).YEndPoints;
    labels2=string(H(2).YData);
    text(xtips2,ytips2,labels2,'HorizontalAlignment','center','FontSize',...
        fontsize1,'FontWeight','bold','VerticalAlignment','bottom');
    
    H(3).FaceColor='#202020';
    xtips3=H(3).XEndPoints;
    ytips3=H(3).YEndPoints;
    labels3=string(H(3).YData);
    text(xtips3,ytips3,labels3,'HorizontalAlignment','center','FontSize',...
        fontsize1,'FontWeight','bold','VerticalAlignment','bottom');
    
    xlabel('Number of Robots','FontSize',fontsize2,'FontWeight','bold');
    ylabel('Performance Gain','FontSize',fontsize2,'FontWeight','bold');
    ylim([0,0.8]);
    set(gca,'fontsize',fontsize2,'FontWeight','bold');
    lgd=legend({'min','mean','max'},'location','northwest');
    lgd.FontSize=fontsize2;
    lgd.FontWeight='bold';
end


