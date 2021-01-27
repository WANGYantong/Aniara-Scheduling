function [ES,LS] = BoundInitialize(NUM_ROBOTS,G_min,D,T)

ES=ones(NUM_ROBOTS,1);
LS=T-sum(G_min,2)-sum(D,2)+1;

% Epsilon=T-1;
% Index=(LS-ES)>=Epsilon-1;
% LS(Index)=Epsilon-1;

end

