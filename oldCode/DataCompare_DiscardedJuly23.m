%% Trying to compare predicted limit surface and data
% Matt Estrada
% July 10 2016
clear
close all
clc

r = 9/2*0.0254; % Distance from COM to back
d = 0.081;      % Distance from surface to ATI
alpha = 11.35;  % [deg]

A = defineGeometry(alpha,r);

trans = @(r) [1 0 0; 0 1 0; r 0 1];

%% Load in simulated limit surface
load('3DscatterLimit.mat')

nonumbers = isnan(FxFyMzWrist(:,1));
FxFyMzWrist(nonumbers,:) = [];

% xlim = 10; 
% ylim = 7; 
% zlim = 0.5;
% [wristx, wristy, wristz] = meshgrid(-xlim:xlim,-ylim:0,-zlim:0.1:zlim)
% wrist = [reshape(wristx,[numel(wristx) 1]) reshape(wristy,[numel(wristx) 1]) reshape(wristz,[numel(wristx) 1])]



Fy = FxFyMzWrist(:,2);
Fy_unit = abs(Fy)/max(abs(Fy))/10
fyn = length(FxFyMzWrist(:,2)); 
scatter3(FxFyMzWrist(:,1),FxFyMzWrist(:,2),FxFyMzWrist(:,3),10,[ Fy_unit Fy_unit ones(fyn,1)])
%scatter3(wrist(:,1),wrist(:,2),wrist(:,3),10)
%legend('Adhesive Capabilities')
%legend('Example of Wrist Compliance Space')

xlabel('F_x [N]')
ylabel('F_y [N]')
zlabel('T_z [Nm]')
title('Sustainable Force/Torque at Wrist')

% %% Load in Data from trial 1
% load('data/2016_07_07_trials1_data')
% 
% % ATI_Fz = -trials(:,4); % normal to object 
% % ATI_Fy = -trials(:,3);  % tangential to object
% % ATI_Mx = trials(:,5);  % moment at base
% 
% ATI_Fz = trials(:,4); % normal to object 
% ATI_Fy = trials(:,3);  % tangential to object
% ATI_Mx = trials(:,5);  % moment at base
% 
% Model_Ftan = -ATI_Fy; 
% Model_Fnor = -ATI_Fz; 
% Model_Tz = ATI_Mx;
% limits_Surf = [Model_Ftan,Model_Fnor,Model_Tz]*[1 0 0; 0 1 0; -.081 0 1]'
% 
% scatter3(limits_Surf(:,1),limits_Surf(:,2),limits_Surf(:,3),60)

%% Load in Data from Trial 2 

twisting = importTrialsOLD('data/2016_07_08_trial2/twisting',-0.081)
scatter3(twisting(:,1),twisting(:,2),twisting(:,3),60,'*')

pull = importTrialsOLD('data/2016_07_08_trial2/Zdir',-0.081)
scatter3(pull(:,1),pull(:,2),pull(:,3),60,'*')

tangential = importTrialsOLD('data/2016_07_08_trial2/Ydir',-0.081)
scatter3(tangential(:,1),tangential(:,2),tangential(:,3),60,'*')

legend('predicted','twisting','Normal','Tangential')

%% Plot in ATI Coordinates
trans = @(r) [1 0 0; 0 1 0; r 0 1];

figure
set(gca,'fontsize',16)
hold on

FxFyMzATI = (trans(0.081)*FxFyMzWrist')'
scatter3(FxFyMzATI(:,1),FxFyMzATI(:,2),FxFyMzATI(:,3),10)

twisting = importTrialsOLD('data/2016_07_08_trial2/twisting',0)
scatter3(twisting(:,1),twisting(:,2),twisting(:,3),60,'*')

pull = importTrialsOLD('data/2016_07_08_trial2/Zdir',0)
scatter3(pull(:,1),pull(:,2),pull(:,3),60,'*')

tangential = importTrialsOLD('data/2016_07_08_trial2/Ydir',0)
scatter3(tangential(:,1),tangential(:,2),tangential(:,3),60,'*')

legend('predicted','twisting','Normal','Tangential')

xlabel('F_x [N]')
ylabel('F_y [N]')
zlabel('T_z [Nm]')
title('Sustainable Force/Torque at ATI')

%% Get a metric on how far off the data is
A_ATI = trans(r+d)*A
A_surf = trans(r)*A;

data_ATI = [twisting;pull]
nData = length(data_ATI);

tensions = zeros(nData,4);

for kk = 1:nData
    tensions(kk,:) = ( lsqnonneg(A_ATI,data_ATI(kk,:)') )'
end

