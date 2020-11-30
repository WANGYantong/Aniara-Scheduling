clear
clc

T=8;
t=1:16;

sequence_A=zeros(1,length(t));
sequence_B=sequence_A;

sequence_A(2:3)=2;
sequence_A(2+T:3+T)=2;
sequence_A(5:7)=4;
sequence_A(5+T:7+T)=4;

sequence_B(2)=5;
sequence_B(2+T)=5;
sequence_B(4:7)=3;
sequence_B(4+T:7+T)=3;

subplot(2,1,1);
stairs(t,sequence_A,'Color','#0072BD');
xticks([1:17]);
xlim([1,17]);
xlabel({'time'});
ylim([0,8]);
ylabel({'rate'});
title('robot 1');

subplot(2,1,2);
stairs(t,sequence_B,'Color','#D95319');
xticks([1:17]);
xlim([1,17]);
xlabel({'time'});
ylim([0,8]);
ylabel({'rate'});
title('robot 2');