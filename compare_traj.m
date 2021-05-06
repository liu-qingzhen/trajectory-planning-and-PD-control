function []=compare_traj(q,t);

n = size(q,1);
names=['123456'];

for i = 1:n
    subplot(n,1,i);
    plot(t(i,:),'linewidth',2.5);
    hold on
    plot(q(i,:),'linewidth',2.5);
    xlim([1 length(q)]) 
    legend('task space','joint space')
    title(['Joint ',names(i),' Positions Trajectory'])
end


  
%     subplot(n,1,6);
%     plot(q(i,:),'linewidth',2.5);
%     title(['Joint ',names(i),' Acceleration Trajectory'])
%     xlim([1 length(q)])
% end
end


