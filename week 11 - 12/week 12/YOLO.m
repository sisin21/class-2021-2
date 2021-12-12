clear; clc;

%%                                                    
rosshutdown

rosinit

% load yolo trained model
pretrained = load('tinyYOLOv2-coco.mat');
detector = pretrained.yolov2Detector;

% change your rostopic image name & msg type
rgb_topic_name='/camera/rgb/image_raw';
depth_topic_name='/camera/depth/image_raw';
topic_type='sensor_msgs/Image';
sub_image=rossubscriber(rgb_topic_name,topic_type);
sub_depth=rossubscriber(depth_topic_name,topic_type);

% yolo trained image size
inputSize = pretrained.yolov2Detector.TrainingImageSize;

h=figure;


while ishandle(h) % if you shutdown figure window, then this code shutdown too

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
   
    % yolo
    [boxes,scores,labels] = detect(detector,im);
    
    %each boxes label
    if ~isempty(boxes) %prevent segmentation error
        label=char(labels);
        depth_value=Depth_extract(boxes,dp);
        score=num2str(scores);
        depth_val=num2str(depth_value);
        label_str={};
        for ii=1:size(scores)
            label_str{ii}=[label(ii,:) ' : ' score(ii,:) ', depth : ' depth_val(ii,:)];
        end
        label_to_im=label_str';
        im = insertObjectAnnotation(im,'rectangle',boxes,label_to_im);
        
    end
    im = imresize(im,sz(1:2));
    imshow(im);
    
    
    
    drawnow;
end