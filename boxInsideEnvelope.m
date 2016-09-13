close all; clear; clc; 
addpath('functionsCvx','functionsHelper','dataGenerated')

r = 9/2*0.0254; % Distance from object COM to object surface
d = 0.081;      % Distance from object surface to ATI
trans = @(r) [1 0 0; 0 1 0; r 0 1];

% % Load in simulated limit surface
% load('3DscatterLimit_paperAsymmetric.mat') % limit variable represents as accelerations about object's COM
% limitWrist = (trans(r)*limit')';
% figure; set(gca,'fontsize',16); hold on;
% scatter3(limitWrist(:,1),limitWrist(:,2),limitWrist(:,3),10) % Predicted Limit

% Load in simulated limit surface
load('dataGenerated/3DscatterLimit_AsymmetricPaper_Sept8') % limit variable represents as accelerations about object's COM
limit(isinf(limit(:,3)),:) = [];% Get rid of erraneous vals
limit(isnan(limit(:,3)),:) = [];% Get rid of erraneous vals
%limitWrist = (trans(r)*limit')';
limitWrist=limit;

figure; set(gca,'fontsize',20); hold on;
plotManualIsolines(limitWrist,limitWrist(:,2))
axis tight


    sx = 10;
    sy = 4.5;
    sz = .15;
    x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-1/2)*sx;
    y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1])*sy;
    z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-1/2)*sz;
    for i=1:6
        h=patch(x(:,i),y(:,i),z(:,i),[47, 98, 53]/256);
        set(h,'edgecolor','k','LineWidth',2,'FaceAlpha',0.5)
    end
%plot3([-5 5],[0 0],[0 0],'b','LineWidth',3)
%plot3([0 0],[0 6],[0 0],'b','LineWidth',3)
%plot3([0 0],[0 0],[-.075 .075],'b','LineWidth',3)

view(3)

% box = [ -5 5; 0 6; -0.08 0.08]; % [xmin xmax; ymin ymax; zmin zmax]
% [wristx, wristy, wristz] = meshgrid(box(1,1):box(1,2),box(2,1):box(2,2),box(3,1):.01:box(3,2));
% wristBox = [reshape(wristx,[numel(wristx) 1]) reshape(wristy,[numel(wristx) 1]) reshape(wristz,[numel(wristx) 1])];
% scatter3(wristBox(:,1),wristBox(:,2),wristBox(:,3),10)
% view(3)

xlabel('F_x [N]')
ylabel('F_y [N]')
zlabel('M_z [Nm]')
%title('Force/Torque at Pivot')
%%
fig = gcf;
fig.PaperPositionMode = 'auto'
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
addpath('functionsCvx','functionsHelper','dataGenerated')
print(fig,'passiveConstraints','-dpdf')
