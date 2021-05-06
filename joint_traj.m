ik = inverseKinematics('RigidBodyTree',e5);
ikWeights = [1 1 1 1 1 1];
traj=[]

for idx=1:length(waypoints)
    ikInitGuess=[-0.5276    0.7811   -1.7791   -0.0000    2.5600    0.5276];
tgtPose = trvec2tform(waypoints(:,idx)');
[config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
ikInitGuess = config;
traj(:,idx)=config
end