load('steplog.txt');
p1 = steplog(:,1:2);
p2 = steplog(:,3:4);
p3 = steplog(:,5:6);
p4 = steplog(:,7:8);

hold on
plot(p1(:,1),p1(:,2),'k')
plot(p2(:,1),p2(:,2),'r')
plot(p3(:,1),p3(:,2),'g')
plot(p4(:,1),p4(:,2),'b')

%%
load('predPreyTypes.csv')
load('predPreyNoTypes.csv')

hold on
% plot(mean(predPreyNoTypes))
% plot(mean(predPreyTypes),'r')
plot(predPreyNoTypes)
plot(predPreyTypes,'r')

legend('blind','types')