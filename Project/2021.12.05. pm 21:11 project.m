close all; clear; clc
addpath('~/catkin_ws/src/mavros/matlab_msg_gen_ros1/glnxa64/install/m')
rosshutdown;
rosinit;
pause(1);
%%
desiredRate= 5;
loopTime =20;
rate = rosrate(desiredRate);
rate.OverrunAction = 'slip';

odomsub = rossubscriber('/mavros/local_position/odom','nav_msgs/Odometry');
imusub = rossubscriber('/mavros/imu/data','sensor_msgs/Imu');
pause(1);
[setpub,setmsg] = rospublisher('/mavros/setpoint_position/local','geometry_msgs/PoseStamped');


for i=1:15
    setmsg.Pose.Position.X = 1;
    setmsg.Pose.Position.Y = 0;
    setmsg.Pose.Position.Z = 1;
    send(setpub,setmsg);
    pause(0.5);
       
    if rem(i,5)==0
        arming = rossvcclient('mavros/cmd/arming');
        testreq1 = rosmessage(arming);
        testreq1.Value=1;
        response1 = call(arming,testreq1,'Timeout',2);
        if testreq1.Value==1
            disp('Arming enabled');
        else
           disp('Arming failed');

        end

        set_mode = rossvcclient('mavros/set_mode');
        testreq2 = rosmessage(set_mode);
        testreq2.CustomMode='OFFBOARD';
        response2 = call(set_mode,testreq2,'Timeout',2);
        if testreq2.CustomMode=='OFFBOARD'
            disp('Offboard enabled');
        else
           disp('Offboard failed');

        end
    end

end

reset(rate);
X=[];
Y=[];
Z=[];


%%
for i = 1:desiredRate*loopTime
    imu = receive(imusub);
    
    if fix((i-1)/(desiredRate*loopTime/4)) == 0
        xd(i)=1;
        yd(i)=(-4)*i/(desiredRate*loopTime/4);
        zd(i)=1;
    end
    if fix((i-1)/25) == 1
        xd(i)=1-4*(i-1*(desiredRate*loopTime/4))/(desiredRate*loopTime/4);
        yd(i)=(-4);
        zd(i)=1;
    end
    if fix((i-1)/25) == 2
        xd(i)=(-3);
        yd(i)=(-4)+4*(i-2*(desiredRate*loopTime/4))/(desiredRate*loopTime/4);
        zd(i)=1;
    end
    if fix((i-1)/25) == 3
        xd(i)=(-3)+4*(i-3*(desiredRate*loopTime/4))/(desiredRate*loopTime/4);
        yd(i)=0;
        zd(i)=1;
    end

    quat = [imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z];
    eul = quat2eul(quat);
    eul2=[eul(1) eul(2) eul(3)];
    if rem(i,25)==0 
        eul2=[eul(1)-pi/2 eul(2) eul(3)];
    end
%     Rot = eul2rotm(eul2);
    trans = eul2quat(eul2);
    
    setmsg.Pose.Position.X = xd(i);
    setmsg.Pose.Position.Y = yd(i);
    setmsg.Pose.Position.Z = zd(i);
    setmsg.Pose.Orientation.X = trans(2);
    setmsg.Pose.Orientation.Y = trans(3);
    setmsg.Pose.Orientation.Z = trans(4);
    setmsg.Pose.Orientation.W = trans(1);
    if rem(i,25)==0 
        pause(5);
    end
    
    xo(i) = 180/pi*eul2(1);
    yo(i) = 180/pi*eul2(2);
    zo(i) = 180/pi*eul2(3);
    
    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n X: %f, Y: %f, Z: %f\n',i,time,xd(i),yd(i),zd(i));
    fprintf('Position X: %f, Y: %f, Z: %f\n',xd(i),yd(i),zd(i));
    fprintf('Euler Angles X: %f, Y: %f, Z: %f\n\n',xo(i),yo(i),zo(i));
    
    send(setpub,setmsg);
    waitfor(rate);


end
 rosshutdown

 %% KSH 2021.12.05. SUN pm 21:11