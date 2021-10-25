function [ES,LS] = BoundUpdate(ST,G_min,G_max,D,T,index)

ET=ST+D(:,index)-1;

ES=ET+G_min(:,index)+1;
LS1=ET+G_max(:,index)+1;
LS2=T-sum(G_min(:,index+1:end),2)-sum(D(:,index+1:end),2)+1;
LS=min([LS1,LS2],[],2);

end

