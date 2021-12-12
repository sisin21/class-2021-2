%% World is project_square.world
%% Depth camera
close all; clear; clc
addpath('~/catkin_ws/src/mavros/matlab_msg_gen_ros1/glnxa64/install/m')
addpath('../gui_test');
gui1();
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

depthsub = rossubscriber('/camera/depth/image_raw', 'sensor_msgs/Image');
imagesub = rossubscriber('/camera/rgb/image_raw', 'sensor_msgs/Image');

pause(1);
[setpub,setmsg] = rospublisher('/mavros/setpoint_position/local','geometry_msgs/PoseStamped');
[velpub,velmsg] = rospublisher('/mavros/setpoint_velocity/cmd_vel','geometry_msgs/TwistStamped');
%%
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
%%
reset(rate);
X=[];
Y=[];
Z=[];

VX = [];
VY = [];
VZ = [];

handles = guidata(gui1);

kp = 0.7;
kd = 2.0;
ki = 0.1;


ptcloud = {};
%%
for i = 1:desiredRate*loopTime
     state = receive(odomsub);
     image_msg = receive(imagesub);
     depth_msg = receive(depthsub);
     imu = receive(imusub);
    
    if fix((i-1)/(desiredRate*loopTime/4)) == 0
        xd(i) = 1;
        yd(i) = (-4)*i/(desiredRate*loopTime/4);
        zd(i) = 1;
        
        vxd(i) = 0;
        vyd(i) = (-4)/(desiredRate*loopTime/4);
        vzd(i) = 0;
    end
    if fix((i-1)/25) == 1
        xd(i) = 1-4*(i-1*(desiredRate*loopTime/4))/(desiredRate*loopTime/4);
        yd(i) = (-4);
        zd(i) = 1;
        
        vxd(i) = (-4)/(desiredRate*loopTime/4);
        vyd(i) = 0;
        vzd(i) = 0;
    end
    if fix((i-1)/25) == 2
        xd(i) = (-3);
        yd(i) = (-4)+4*(i-2*(desiredRate*loopTime/4))/(desiredRate*loopTime/4);
        zd(i) = 1;
        
        vxd(i) = 0;
        vyd(i) = 4/(desiredRate*loopTime/4);
        vzd(i) = 0;
    end
    if fix((i-1)/25) == 3
        xd(i)=(-3)+4*(i-3*(desiredRate*loopTime/4))/(desiredRate*loopTime/4);
        yd(i)=0;
        zd(i)=1;
        
        vxd(i) = 4/(desiredRate*loopTime/4);
        vyd(i) = 0;
        vzd(i) = 0;
    end
    
    quat = [imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z];
    
    if rem(i,25)==1
        eul = quat2eul(quat);
        eul2=[eul(1)-pi/2 eul(2) eul(3)];
        trans = eul2quat(eul2);
        setmsg.Pose.Orientation.X = trans(2);
        setmsg.Pose.Orientation.Y = trans(3);
        setmsg.Pose.Orientation.Z = trans(4);
        setmsg.Pose.Orientation.W = trans(1);
    end
    

    
    
    setmsg.Pose.Position.X = xd(i);
    setmsg.Pose.Position.Y = yd(i);
    setmsg.Pose.Position.Z = zd(i);
    
    view = quat2eul(quat);
%     view = [180/pi*view1(1),180/pi*view(2),180/pi*view(3)];
    
    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n X: %f, Y: %f, Z: %f\n',i,time,xd(i),yd(i),zd(i));
    fprintf('Position X: %f, Y: %f, Z: %f\n',xd(i),yd(i),zd(i));
    fprintf('Orentation X: %f, Y: %f, Z: %f\n\n',view(3),view(2),view(1));
    %%
    
     [img, im_alpha] = readImage(image_msg);
     [dep, dp_alpha] = readImage(depth_msg);

     
      x=state.Pose.Pose.Position.X;
      X=[X;x];
      y=state.Pose.Pose.Position.Y;
      Y=[Y;y];
      z=state.Pose.Pose.Position.Z;
      Z=[Z;z];
    
    vx=state.Twist.Twist.Linear.X;
    VX=[VX;vx];
    vy=state.Twist.Twist.Linear.Y;
    VY=[VY;vy];
    vz=state.Twist.Twist.Linear.Z;
    VZ=[VZ;vz];
    
    Current.x = X';
    Current.y = Y';
    Current.z = Z';
    
    set(handles.datax, 'String', x);
    set(handles.datay, 'String', y);
    set(handles.dataz, 'String', z);
    
    set(handles.datavx, 'String', vx);
    set(handles.datavy, 'String', vy);
    set(handles.datavz, 'String', vz);
    
   handles.hPlot1 = plot3(handles.axes1, NaN, NaN, NaN);
   set(handles.hPlot1, 'XData', [get(handles.hPlot1, 'XData') Current.x], ...
                       'YData', [get(handles.hPlot1, 'YData') Current.y], ...
                       'ZData', [get(handles.hPlot1, 'ZData') Current.z], ...
                       'MarkerEdgeColor', 'r');
%    plot code;
%  
    axis(handles.axes1, [-5 2 -6 1 -2 4]);
    
    imshow(img, 'Parent', handles.axes2);
    imagesc(dep, 'Parent', handles.axes3);
    
    drawnow;
        
%     setmsg.Pose.Position.X = xd(i);
%     setmsg.Pose.Position.Y = yd(i);
%     setmsg.Pose.Position.Z = zd(i);
%     
%     intx = 0;
%     inty = 0;
%     intz = 0;
%     
%     ex = xd(i)-x;
%     ey = yd(i)-y;
%     ez = zd(i)-z;
% 
%     if z>0.3
%         intx=intx+ex;
%         if abs(intx)>2
%             intx=sign(intx)*2;
%         end
%         
%         inty=inty+ey;
%         if abs(inty)>2
%             inty=sign(inty)*2;
%         end
%         
%         intz=intz+ez;
%         if abs(intz)>1
%             intz=sign(intz)*1;
%         end
%     end
%     
%     cmd.x=ki * intx + kp * ex + kd * (vxd(i) - vx);
%     cmd.y=ki * inty + kp * ey + kd * (vyd(i) - vy);
%     cmd.z=0.05 * intz + kp * ez + 0.15 * (vzd(i) - vz);
% 
%     velmsg.Header.Stamp=rostime('now');
%     velmsg.Twist.Linear.X= cmd.x;
%     velmsg.Twist.Linear.Y= cmd.y;
%     velmsg.Twist.Linear.Z = cmd.z;
    
    scale = 1/2;
     
     re_img = imresize(img, scale);
     re_dep = imresize(dep, scale);
 
    K=([565.6008952774197, 0.0, 320.5; 0.0, 565.6008952774197, 240.5; 0.0, 0.0, 1.0]);

    cloud=[];
    dep_mm=re_dep*1000;
    Sd=size(dep_mm);
    [pX, pY]=meshgrid(1:Sd(2),1:Sd(1));

    pX=pX-K(1,3)*scale+0.5;
    pY=pY-K(2,3)*scale+0.5;

    xDf=double(dep_mm/(K(1,1)*scale));
    yDf=double(dep_mm/(K(2,2)*scale));

    pX=pX.*xDf;
    pY=pY.*yDf;

    pXY=cat(3,pX,pY);

    cloud=cat(3,pXY,dep_mm);
    cloud=reshape(cloud,[],3)/1000;

%     R=quat2rotm(quat);
% %     R1=[R t]*[cloud(k,:):1];
%     eul1 = [pi/2 0 pi/2];
%     rotmZYX = eul2rotm(eul1);
%     cloud=R*rotmZYX;

    ptcloud{i} = cloud;
    save('ptcloud.mat',cloud);
% figure(1)
    plot3(X, Y, Z, 'LineWidth', 2, 'Color', 'b', 'parent', handles.axes1);
    hold(handles.axes1, 'on');
%     plot3(xd(1:i), yd(1:i), zd(1:i), 'LineWidth', 2, 'Color', 'r', 'parent', handles.axes1);
    plot3(cloud(:,3), cloud(:,1), -cloud(:,2), 'ok', 'MarkerSize', 1, 'parent', handles.axes1);
    hold(handles.axes1, 'off');
    grid(handles.axes1, 'on');
    axis(handles.axes1, [-5 2 -6 1 -2 4]);

% subplot(3,2,1); plot(t, X,'-b',t, xd,'-r','LineWidth', 2);
% subplot(3,2,3); plot(t, Y,'-b',t, yd,'-r','LineWidth', 2);
% subplot(3,2,5); plot(t, Z,'-b',t, zd,'-r','LineWidth', 2);
% 
% subplot(3,2,2); plot(t, VX,'-b',t, vxd,'-r','LineWidth', 2);
% subplot(3,2,4); plot(t, VY,'-b',t, vyd,'-r','LineWidth', 2);
% subplot(3,2,6); plot(t, VZ,'-b',t, vzd,'-r','LineWidth', 2);
% 
% plot3(X, Y, Z, '-b', xd, yd, zd, '-r', 'LineWidth', 2);
    
    send(setpub,setmsg);
    waitfor(rate);
end
rosshutdown
 %% KSH 2021.12.05. SUN pm 23:32
 %% added cell array {point cloud(not work?)} & velocity