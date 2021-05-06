function [q,qd,qdd]=polynomialtraj(waypoints,waypointsVel,waypointTimes,ts)
theta0=0
qn=size(waypoints,1);
q=waypoints(:,1);
qd=zeros(qn,1);
qdd=zeros(qn,1);
for i =1:size(waypoints,2)-1
    t0=waypointTimes(i);
    tf=waypointTimes(i+1);
    t=(t0:ts:tf)-t0;
    a0=waypoints(:,i);
    a1=waypointsVel(:,i);
    a2=3/((tf-t0).^2)*(waypoints(:,i+1)-waypoints(:,i))-2/(tf-t0)*waypointsVel(:,i)-1/(tf-t0)*waypointsVel(:,i+1)
    a3=-2/((tf-t0).^3)*(waypoints(:,i+1)-waypoints(:,i))+1/((tf-t0).^2)*(waypointsVel(:,i+1)+waypointsVel(:,i));
    x=a0+a1*t+a2*t.^2+a3*t.^3;
    v=a1+2*a2*t+3*a3*t.^2;
    a=2*a2+6*a3*t;
    q=[q,x(:,2:end)];
    qd=[qd,v(:,2:end)];
    qdd=[qdd,a(:,2:end)];
end
end
