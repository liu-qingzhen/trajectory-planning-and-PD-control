function []=plottrajectory(q,qd,qdd);

n = size(q,1);
if n==3
    names=['XYZ'];
else
    names=['123456']
end
for i = 1:n
    figure
    subplot(n,1,1);
    plot(q(i,:),'linewidth',2.5);
    xlim([1 length(q)])
    
    title([names(i),' Positions Trajectory'])
    subplot(n,1,2);
    plot(qd(i,:),'linewidth',2.5);
    title([names(i),' Velocity Trajectory'])
    xlim([1 length(q)])
    
    subplot(n,1,3);
    plot(q(i,:),'linewidth',2.5);
    title([names(i),' Acceleration Trajectory'])
    xlim([1 length(q)])
end
end


