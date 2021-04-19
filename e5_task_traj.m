clear
clc

e5waypoints

waypointVels = 0.1*[[0;0;0],rand(3,3),[0;0;0]];
% waypointVels=[0    0.0532    0.0804    0.0146         0;
%           0    0.0465    0.0424    0.0524         0;
%           0    0.0277    0.0070    0.0286         0]
% 1st

% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));
% 1st      0    0.0532    0.0804    0.0146         0
%          0    0.0465    0.0424    0.0524         0
%          0    0.0277    0.0070    0.0286         0


% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;
ik = inverseKinematics('RigidBodyTree',e5);
ikWeights = [1 1 1 1 1 1];

%% Generate trajectory

traj_type='3th-order Polynomial'
switch traj_type
    case 'Cubic Polynomial'
        [q,qd,qdd]=polynomialtraj(waypoints,waypointTimes,ts);
    case '3th-order Polynomial'
        [q,qd,qdd]=CubicPolynomial_viaPoints(waypoints,waypointVels,waypointTimes,ts);
    case '5th-order Polynomial'
        [q,qd,qdd]=HigherOrderPolynomial(waypoints,waypointVels,waypointAccels,waypointTimes,ts)
end
plottrajectory(q,qd,qdd);
%%
figure
show(e5,ikInitGuess,'Frames','off','PreservePlot',false);
zlim([-0.5 1.5])
hold on
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ko','linewidth',5);
hold on
plot3(q(1,:),q(2,:),q(3,:),'marker','o','color','#0072BD');
% title('3-order Polynomial')
hold on
% show(e5,ikInitGuess,'Frames','off','PreservePlot',false);
pic_num=1;
for idx = 1:numel(trajTimes)
    % Solve IK
    tgtPose = trvec2tform(q(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    idx
    
    % Show the robot
    
    show(e5,config,'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow
    
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map,['taskspace_trajectory','_',traj_type,'.gif'],'gif', 'Loopcount',inf,'DelayTime',0.2);
    else
        imwrite(I,map,['taskspace_trajectory','_',traj_type,'.gif'],'WriteMode','append','DelayTime',0.2);
    end
    pic_num = pic_num + 1;
    
end










