close all; clear; clc;

x = linspace(-2,2);
ranges = abs((1.5).*x.^2 + 5);
ranges(45:55) = 3.5;
angles = linspace(-pi/2,pi/2,numel(ranges));

scan = lidarSan(ranges,angles);
plot(scan)

minRange = 0.1;
maxRange = 7;
scan2 = removeInvalidData(scan,'RangeLimits',[minRange maxRange]);
hold on
plot(scan2)
legend('All Points','Valid Points')

%%
refRanges = 5*ones(1,300);
refAngles = linspace(-pi/2,pi/2,300);
refScan = lidarScan(refRanges,refAngles);

transformedScan = transformScan(refScan,[0.5 0.2 0]);

rotateScan = transformScan(refScan,[0,0,deg2rad(20)]);
%%
refRanges = 5*ones(1,300);
refAngles = linspace(-pi/2,pi/2,300); 
refScan = lidarScan(refRanges,refAngles);

currScan = transformScan(refScan,[0.5 0.2 0]);

pose = matchScans(currScan,refScan);

currScan2 = transformScan(currScan,pose);

subplot(2,1,1);
hold on
plot(currScan)
plot(refScan)
title('Original Scans')
hold off

subplot(2,1,2);
hold on
plot(currScan2)
plot(refScan)
title('Aligned Scans')
xlim([0 5])
hold off