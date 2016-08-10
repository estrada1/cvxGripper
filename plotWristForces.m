function plotWristForces( files, offset_nor, offset_tan, k, box, pivotLateral)
%% Plotting wrist forces measured by the ATI sensor 
% Matt Estrada
% directory of files
% files - array of file names
% pivotOffset - normal to the gripper
% box - 
% pivotLateral- tangential to the gripper 

[mTrials n] = size(files); 


figure; set(gca,'fontsize',16); hold on;

for nn = 1:mTrials
    thisTrial = movingPivot(importATI(files{nn,2},offset_nor, offset_tan),k);
    scatter3(thisTrial(:,1),thisTrial(:,2),thisTrial(:,3),60,'.')
end

[wristx, wristy, wristz] = meshgrid(box(1,1):box(1,2),box(2,1):box(2,2),box(3,1):.01:box(3,2));
wristBox = [reshape(wristx,[numel(wristx) 1]) reshape(wristy,[numel(wristx) 1]) reshape(wristz,[numel(wristx) 1])];
scatter3(wristBox(:,1),wristBox(:,2),wristBox(:,3),10)
view(3)

xlabel('F_x [N]')
ylabel('F_y [N]')
zlabel('T_z [Nm]')
%title('Force/Torque at Pivot')
legend(files{:,1})

%% Pulling in a bunch of directions 

% % Pulling Normal to Gripper +- 0.1 Nm, 
% figure; set(gca,'fontsize',16); hold on
% sweep = importATI('data/2016_07_27_TestingWristCompliance/05_sweep',pivotOffset);
% trial1 = movingPivot(sweep,k); 
% 
% scatter3(trial1(:,1),trial1(:,2),trial1(:,3),'.')
% 
% if ~isempty(box)
%     xlim = 5.5; 
%     ylim = 12; 
%     zlim = 0.25;
%     [wristx, wristy, wristz] = meshgrid(-xlim:xlim,0:ylim,-zlim:0.1:zlim)
%     wristBox = [reshape(wristx,[numel(wristx) 1]) reshape(wristy,[numel(wristx) 1]) reshape(wristz,[numel(wristx) 1])]
%     scatter3(wristBox(:,1),wristBox(:,2),wristBox(:,3),10)
% end
% 
%     title('Forces at ATI')
%     xlabel('F_x [N]')
%     ylabel('F_y [N]')
%     zlabel('T_z [Nm]')
%     title('Range of Motion Forces at Pivot')
    
%% Pulling in a bunch of directions 

% Pulling Normal to Gripper +- 0.1 Nm, 
% figure; set(gca,'fontsize',16); hold on
% sweep = importATI('data/2016_07_27_TestingWristCompliance/05_sweep',pivotOffset)
% trial1 = movingPivot(sweep,k); 
% 
% scatter3(trial1(:,1),trial1(:,2),trial1(:,3),'.')
% 
% xlim = 5.5; 
% ylim = 12; 
% zlim = 0.25;
% [wristx, wristy, wristz] = meshgrid(-xlim:xlim,0:ylim,-zlim:0.1:zlim)
% wristBox = [reshape(wristx,[numel(wristx) 1]) reshape(wristy,[numel(wristx) 1]) reshape(wristz,[numel(wristx) 1])]
% scatter3(wristBox(:,1),wristBox(:,2),wristBox(:,3),10)
% 
%     title('Forces at ATI')
%     xlabel('F_x [N]')
%     ylabel('F_y [N]')
%     zlabel('T_z [Nm]')
%     title('Range of Motion Forces at Pivot')

%% Pulling in a bunch of directions 

% % Pulling Normal to Gripper +- 0.1 Nm, 
% figure; set(gca,'fontsize',16); hold on
% sweep = importATI('data/2016_07_27_TestingWristCompliance/05_sweep',pivotOffset)
% trial1 = (sweep); 
% 
% scatter3(trial1(:,1),trial1(:,2),trial1(:,3),'.')
% 
% xlim = 5.5; 
% ylim = 12; 
% zlim = 0.25;
% [wristx, wristy, wristz] = meshgrid(-xlim:xlim,0:ylim,-zlim:0.1:zlim)
% wristBox = [reshape(wristx,[numel(wristx) 1]) reshape(wristy,[numel(wristx) 1]) reshape(wristz,[numel(wristx) 1])]
% scatter3(wristBox(:,1),wristBox(:,2),wristBox(:,3),10);
% 
%     title('Forces at ATI')
%     xlabel('F_x [N]')
%     ylabel('F_y [N]')
%     zlabel('T_z [Nm]')
%     title('Without Correction')
%     
end