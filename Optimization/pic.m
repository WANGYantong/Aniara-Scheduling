clear
clc

%% Data Loading
ROBOTS=2:2:10;
TASKS=1:3;

NoScheduling_mean=zeros(length(ROBOTS),length(TASKS));
RTWPA_mean=NoScheduling_mean;
Optimal_mean=NoScheduling_mean;

NoScheduling_time=zeros(length(ROBOTS),length(TASKS));
RTWPA_time=NoScheduling_time;
Optimal_time=NoScheduling_time;

for ii=1:length(ROBOTS)
    for jj=1:length(TASKS)
        filename="data_2021/dataset"+ROBOTS(ii)+"_"+TASKS(jj)+".mat";
        buff=load(filename);
        
        NoScheduling_mean(ii,jj)=mean(buff.random_max);
        RTWPA_mean(ii,jj)=mean(buff.multiR_max);
        Optimal_mean(ii,jj)=mean(buff.optimal_max);
        
        NoScheduling_time(ii,jj)=mean(buff.random_time);
        RTWPA_time(ii,jj)=mean(buff.multiR_time);
        Optimal_time(ii,jj)=mean(buff.optimal_time);
    end
end

%% Figure Drawing
curve_LineWidth=5.6;
curve_MarkerSize=20;
box_LineWidth=3.6;
box_FontSize=42;
lgd_FontSize=40;

for jj=1:length(TASKS)
    figure(jj);
    hold on;
    plot(ROBOTS,NoScheduling_mean(:,jj),"-o",...
        'LineWidth',curve_LineWidth,'MarkerSize',curve_MarkerSize);
    plot(ROBOTS,RTWPA_mean(:,jj),"-x",...
        'LineWidth',curve_LineWidth,'MarkerSize',curve_MarkerSize);
    plot(ROBOTS,Optimal_mean(:,jj),"-s",...
        'LineWidth',curve_LineWidth,'MarkerSize',curve_MarkerSize);
    hold off;

    xlabel('Number of Robots');
    ylabel('Peak Data Rate (Mbps)');
%     ylim('padded');
    ylim=get(gca,'ylim');
    ylim(1)=0;

    set(gca,'XTick',ROBOTS,'LineWidth',box_LineWidth,'ylim',ylim,...
    'FontSize',box_FontSize,'FontWeight','bold');
    lgd=legend({'No-Scheduling','RTWPA','Optimal'},'Orientation','vertical',...
    'Location','northwest','FontSize',lgd_FontSize);
    grid minor;
    box on;
end

flow=[buff.random_table{3}(end,:);
      buff.multiR_table{3}(end,:);
      buff.optimal_table{3}(end,:)];

figure(jj+1);
hold on;
for ii=1:3
    stairs(0:14,flow(ii,:),...
        'LineWidth',curve_LineWidth,'MarkerSize',curve_MarkerSize);
end
hold off;

xlabel('Time Slots');
ylabel('Data Rate (Mbps)');
xlim([0 14-1e-2]);
ylim('padded');
set(gca,'LineWidth',box_LineWidth,...
    'FontSize',box_FontSize,'FontWeight','bold');
lgd=legend({'No-Scheduling','RTWPA','Optimal'},'Orientation','vertical',...
    'Location','northwest','FontSize',lgd_FontSize);
grid minor;
box on;