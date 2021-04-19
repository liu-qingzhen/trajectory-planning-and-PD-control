clear
clc

%%

rootDir = fileparts(which(mfilename));
addpath(genpath('matlab'),genpath('simulink'),genpath('utilities'));

load e5

e5waypoints;

eeName='elfin_end_link';
numJoints = numel(e5.homeConfiguration);
% ikInitGuess = [-1.1829    0.8498    2.3262    0.0000   -1.4764    1.1829];
ikInitGuess=[-0.5276    0.7811   -1.7791   -0.0000    2.5600    0.5276];
tform = getTransform(e5,ikInitGuess,eeName,'world');
endeff=tform2trvec(tform)
ik = inverseKinematics('RigidBodyTree',e5);
ikWeights = [1 1 1 1 1 1];



for idx=1:numel(waypointTimes)
    cfg = trvec2tform(waypoints(:,idx)');
    [config,info] = ik(eeName,cfg,ikWeights,ikInitGuess);
%      ikInitGuess = config;
    jointcfg(:,idx)=config';
end

%%
waypointVels =[0    0.1102    0.2629    0.0890         0;
         0    0.0596    0.0884    0.2246         0;
         0    0.2625    0.2455    0.2307         0;
         0    0.1207    0.1670    0.1413         0;
         0    0.0030    0.0484    0.0572         0;
         0    0.0971    0.1650    0.0426         0];
% waypointVels=[0    0.0532    0.0804    0.0146         0;
%           0    0.0465    0.0424    0.0524         0;
%           0    0.0277    0.0070    0.0286         0]
% 1st

% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));
% 1st      0    0.0532    0.0804    0.0146         0
%          0    0.0465    0.0424    0.0524         0
%          0    0.0277    0.0070    0.0286         0

%% JOINT SPACE
numJoints=6;
% [qJoint,qdJoint,qddJoint] = polynomialtraj(jointcfg,waypointTimes,ts);
tic
traj_type='5th-order Polynomial'
switch traj_type
    case 'Cubic Polynomial zero'
        [q,qd,qdd]=polynomialtraj(jointcfg,waypointTimes,ts);
    case '3th-order Polynomial non-zero'
        [q,qd,qdd]=CubicPolynomial_viaPoints(jointcfg,waypointVels,waypointTimes,0.1);
    case '5th-order Polynomial'
        [q,qd,qdd]=HigherOrderPolynomial(jointcfg,waypointVels,waypointAccels,waypointTimes,ts);
end
jointtime=toc;
display(['joint space time ',num2str(jointtime),'s']);


% plot_joint_trajectory(q);

jointpoints=[]
for i =1:size(q,2)
    tform=getTransform(e5,q(:,i)',eeName);
    jointpoints(:,i)=tform2trvec(tform);
end

%%
open_system('control_simulink.slx');
simout = sim('control_simulink.slx');

% e5viz.ShowMarker = true;
% e5viz.showFigure;
rateCtrlObj = rateControl(length(simout.tout)/(max(simout.tout)));
show(e5,config,'Frames','off','PreservePlot',false);
hold on
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ko','LineWidth',2);
hold on
pic_num=1
for i = 1:length(simout.tout)
    config = simout.cfg.data(i,:);
    show(e5,config,'Frames','off','PreservePlot',false);
    codi=getTransform(e5,config,eeName,'world');
    plot3(codi(1,4),codi(2,4),codi(3,4),'x','LineWidth',0.3);
    %     plot3(e5viz.MarkerBodyPose(1,4),e5viz.MarkerBodyPose(2,4),e5viz.MarkerBodyPose(3,4));
    %     hold on
    waitfor(rateCtrlObj);
    drawnow
     F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map,'simulink.gif','gif', 'Loopcount',inf,'DelayTime',0.05);
    else
        imwrite(I,map,'simulink.gif','gif','WriteMode','append','DelayTime',0.05);
    end
    pic_num = pic_num + 1;

end