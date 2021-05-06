function []=plottrajectory(q,qd,qdd);

n = size(q,1);
names=['123456'];

for i = 1:n
    
    subplot(n,1,i);
    plot(q(i,:),'linewidth',2.5);
    xlim([1 length(q)]) 
    title(['Joint ',names(i),' Positions Trajectory'])
end
figure
for i = 1:n

    subplot(n,1,i);
    plot(qd(i,:),'linewidth',2.5);
    xlim([1 length(q)]) 
    title(['Joint ',names(i),' Velocity Trajectory'])
end 
figure
for i = 1:n

    subplot(n,1,i);
    plot(qdd(i,:),'linewidth',2.5);
    xlim([1 length(q)]) 
    title(['Joint ',names(i),' Acceleration Trajectory'])
end
    %     subplot(n,1,2);
%     plot(qd(i,:),'linewidth',2.5);
%     title(['Joint ',names(i),' Velocity Trajectory'])
%     xlim([1 length(q)])
%     
%     subplot(n,1,3);
%     plot(q(i,:),'linewidth',2.5);
%     title(['Joint ',names(i),' Acceleration Trajectory'])
%     xlim([1 length(q)])
%     
%     subplot(n,1,4);
%     plot(q(i,:),'linewidth',2.5);
%     xlim([1 length(q)])
%     title(['Joint ',names(i),' Positions Trajectory'])
%     
%     subplot(n,1,5);
%     plot(qd(i,:),'linewidth',2.5);
%     title(['Joint ',names(i),' Velocity Trajectory'])
%     xlim([1 length(q)])
%     
%     subplot(n,1,6);
%     plot(q(i,:),'linewidth',2.5);
%     title(['Joint ',names(i),' Acceleration Trajectory'])
%     xlim([1 length(q)])
% end
end


