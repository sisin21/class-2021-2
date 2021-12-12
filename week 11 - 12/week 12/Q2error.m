close all; clear; clc
rosshutdown;

addpath('~/catkin_ws/src/mavros/matlab_msg_gen_ros1/glnxa64/install/m')
rosinit
pause(1);
addpath('../../gui_test')
gui1();
%%
desiredRate = 5;
loopTime =30;
rate = rosrate(desiredRate);
rate.OverrunAction = 'slip';

odomsub =rossubscriber('/mavros/local_position/odom','nav_msgs/Odometry');
imusub =rossubscriber('/mavros/imu/data','sensor_msgs/Imu');

depthsub =rossubscriber('/camera/depth/image_raw','sensor_msgs/Image');
imagesub =rossubscriber('/camera/rgb/image_raw','sensor_msgs/Image');
%%
% load yolo trained model
pretrained = load('tinyYOLOv2-coco.mat');
detector = pretrained.yolov2Detector;
inputSize = pretrained.yolov2Detector.TrainingImageSize;
rgb_topic_name='/camera/rgb/image_raw';
depth_topic_name='/camera/depth/image_raw';
topic_type='sensor_msgs/Image';
sub_image=rossubscriber(rgb_topic_name,topic_type);
sub_depth=rossubscriber(depth_topic_name,topic_type);
inputSize = pretrained.yolov2Detector.TrainingImageSize;
%%

pause(1);
[setpub,setmsg] = rospublisher('/mavros/setpoint_position/local','geometry_msgs/PoseStamped');
K=([565.6008952774197, 0.0, 320.5; 0.0, 565.6008952774197, 240.5; 0.0, 0.0, 1.0]);

reset(rate);
X=[];
Y=[];
Z=[];
VX=[];
VY=[];
VZ=[];

handles = guidata(gui1);

for i = 1:desiredRate*loopTime
    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)
    
    state = receive(odomsub);
    imu = receive(imusub);
    
    quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z]; 
    eul = quat2eul(quat);
    eul=[-eul(1) -eul(2) eul(3)];
    Rot=eul2rotm(eul);
    image_msg=receive(imagesub);
    depth_msg=receive(depthsub);

    [img,im_alpha] = readImage(image_msg);
     [dep,dp_alpha] = readImage(depth_msg);

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
    
    trans=[x;y;z];
    
    
    set(handles.datax, 'String', x);
    set(handles.datay, 'String', y);
    set(handles.dataz, 'String', z);
    set(handles.datavz, 'String', vx);
    set(handles.datavy, 'String', vy);
    set(handles.datavz, 'String', vz);
        %%
	image=receive(sub_image);
    depth=receive(sub_depth);
    img=readImage(image); % read rosmsg to image
    dep=readImage(depth);
    dp = imresize(dep,inputSize);
    sz=size(img);
    
    if numel(img)==sz(1)*sz(2) % when image data type is grayscale(mono8 type)
        Image = cat(3, img, img, img);
        im = imresize(Image,inputSize);
    else
        im = imresize(img,inputSize);
    end
    [boxes,scores,labels] = detect(detector,im);
    if ~isempty(boxes) %prevent segmentation error
        label=char(labels);
        depth_value=Depth_extract(boxes,dp);
        score=num2str(scores);
        depth_val=num2str(depth_value);
    end
    %%
    
    % resize image
    scale=1/1;
    re_img=imresize(img,scale);
    re_dep=imresize(dep,scale);
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
    cloud_nan=cat(3,pXY,dep_mm);
    cloud_nan=reshape(cloud_nan,[],3)/1000;
    cloud = rmmissing(cloud_nan);
    n=length(cloud);
    cam_eul=[pi/2+pi 0 pi/2+pi];
    rotcam=eul2rotm(cam_eul);
    cloud_affine=[];
    cloud_affine=([Rot trans]*[rotcam zeros(3,1);0 0 0 1]*[cloud';ones(1,n)])';
    ptCloud=pointCloud(cloud_affine);
    ptCloud_d=pcdownsample(ptCloud,'gridAverage',0.1);
    [groundPtsIdx,nonGroundPtCloud,groundPtCloud] = segmentGroundSMRF(ptCloud_d);
    ptCloud_obs = rmmissing(nonGroundPtCloud.Location);
    x_mean=mean(ptCloud_obs(:,1));
    set(handles.ox, 'String', x_mean);
    y_mean=mean(ptCloud_obs(:,2));
    set(handles.oy, 'String', y_mean);
    z_mean=mean(ptCloud_obs(:,3));
    set(handles.oz, 'String', z_mean);
    depth_val=str2num(depth_val);
    plot3(X,Y,Z,'LineWidth',2,'Color', 'b','parent',handles.axes1); hold( handles.axes1, 'on' )
    plot3(ptCloud_obs(:,1),ptCloud_obs(:,2),ptCloud_obs(:,3),'ok','MarkerSize',1,'parent',handles.axes1); 
    plot3([depth_val depth_val depth_val depth_val depth_val], [boxes(:,1) boxes(:,3) boxes(:,3) boxes(:,1) boxes(:,1)], [boxes(:,2) boxes(:,2) boxes(:,4) boxes(:,4) boxes(:,2)],'LineWidth',2,'Color','r','parent',handles.axes1);
    hold(handles.axes1, 'off' );    grid(handles.axes1,'on');
%     axis(handles.axes1,[-1 10 -4 4 -1 3]);

    xlabel(handles.axes1,'x');
    ylabel(handles.axes1,'y');
    zlabel(handles.axes1,'z');

    imshow(img,'Parent',handles.axes2);
    imagesc(dep,'Parent',handles.axes3);
    if mod(i,2) == 1
        xd=[4;1];
        xx(:,1)=[0;0];
        vxx(:,1)=[0;0];

        bx=x_mean; % obstacle position x
        by=y_mean;  % obstacle position y
        obs=[bx;by];

        dt=0.05;
        t(1)=0;
        ns=size(t,2);
        flag=1;
        j=1;
        r_rho=2.0; % infludence of the obstacle
        eps=0.2;
        dphi_r=[0;0];

        a=0.5; % potential gain
        k=0.1;
        while(flag)
            dphi_a=-(xx(:,j)-xd);
            r_obs=norm(xx(:,j)-obs);
            r_goal=norm(xx(:,j)-xd);

            n=2;
            if r_obs<r_rho
                dphi_r=0.5*0.1*power((1/(r_obs-0.2))-(1/r_rho),2)*power(r_goal,3);
                %your code here
            else
                 dphi_r=0;
            end


            dphi_p(:,j)=dphi_a-dphi_r;
            dphi_p(:,j)=a*dphi_p(:,j)/(norm(dphi_p(:,j)));
            ndphi(j)=(norm(dphi_p(:,j)));
            vxx(:,j+1)=(dphi_p(:,j));
            xx(:,j+1)=xx(:,j)+(dphi_p(:,j))*dt;
            t(j+1)=t(j)+dt;
            if (norm(xx(:,j+1)-xd)<0.1)
                break;
                X=['Iteration No.: ',num2str(i), ', Arrival time: ', num2str(t(i+1)),'s'];
                disp(X);
            end

            j=j+1;
        end
    end
    j= round(i/desiredRate);
    pX=[];
    pY=[];
    pZ=[];
        
    send(setpub,setmsg);
    waitfor(rate);


end
 rosshutdown