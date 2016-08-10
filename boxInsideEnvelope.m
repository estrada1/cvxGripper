
r = 9/2*0.0254; % Distance from object COM to object surface
d = 0.081;      % Distance from object surface to ATI
trans = @(r) [1 0 0; 0 1 0; r 0 1];

% Load in simulated limit surface
load('3DscatterLimit_paperAsymmetric.mat') % limit variable represents as accelerations about object's COM
limitWrist = (trans(r)*limit')';


figure; set(gca,'fontsize',16); hold on;
scatter3(limitWrist(:,1),limitWrist(:,2),limitWrist(:,3),10) % Predicted Limit

box = [ -5 5; 0 6; -0.08 0.08]; % [xmin xmax; ymin ymax; zmin zmax]
[wristx, wristy, wristz] = meshgrid(box(1,1):box(1,2),box(2,1):box(2,2),box(3,1):.01:box(3,2));
wristBox = [reshape(wristx,[numel(wristx) 1]) reshape(wristy,[numel(wristx) 1]) reshape(wristz,[numel(wristx) 1])];
scatter3(wristBox(:,1),wristBox(:,2),wristBox(:,3),10)
view(3)

xlabel('F_x [N]')
ylabel('F_y [N]')
zlabel('T_z [Nm]')
title('Force/Torque at Pivot')
