%% Plotting wrist forces measured by the ATI sensor 
% Matt Estrada

clear; clc; close all; 
addpath('functionsCvx','functionsHelper','dataGenerated')

% pivotOffset = -0.168;     % [m] Distance to pivot from ATI 
% pivotOffset = -.015         % This seems to give better numbers for first
% trials

offset_nor = -0.13;     % [m] Distance to pivot from ATI 
offset_tan = 0.0159;    % [m] Pivot is mounted a little off 

% Stiffness of normal direction 
stiffness = [.01 1.2; .02 2.5; .03 3.5; 0.04 4.56];
polyfit(stiffness(:,1),stiffness(:,2),1);
k = stiffness(:,1)\stiffness(:,2);

directory = 'data/2016_07_27_TestingWristCompliance/';
files = {...
        'moments', [directory '02_moments'];
        'pulling', [directory '03_normal'];
        'tangential', [directory '04_tangential'];
        %'sweep', [directory '05_sweep'];
        %'forces', [directory '07_forces'];
        };
pivotLateral = 0; 

%box = [ -5 5; 0 6; -0.08 0.08]; % [xmin xmax; ymin ymax; zmin zmax]
bounds = [10 6 .15];

plotWristForces(files, offset_nor, offset_tan, k, bounds, pivotLateral)
title('Force/Torque at Pivot')

%% Fitting parameters

% Tangential Offset fit
directory = 'data/2016_07_27_TestingWristCompliance/';
filename = [directory '03_normal'];
data = importATI(filename,offset_nor,offset_tan);
offset = data(:,2)\data(:,3);

figure; set(gca,'fontsize',16); hold on;
%thisTrial = movingPivot(importATI(files{nn,2},pivotOffset),k);
scatter3(data(:,1),data(:,2),data(:,3),60,'.')
scatter3(data(:,1)*0,data(:,2),data(:,2)*offset,60,'.')

xlabel('F_x [N]')
ylabel('F_y [N]')
zlabel('T_z [Nm]')
title('Force/Torque at Pivot')
legend('Pulling in Normal Direction', 'Fit')

% Stiffness fit
figure
plot(0:.01:.05, (0:.01:.05)*k, stiffness(:,1),stiffness(:,2),'*')
title('Normal Axis Stiffness Fit')

%% Defunct Code, Shiquan's suggestion 
% figure
% yo = unique([-1 -1 -1; perms([1 1 -1]); perms([1 -1 -1]); 1 1 1], 'rows');
% yo2 = delaunay3(yo(:,1), yo(:,2), yo(:,3), {'Qt', 'Qbb', 'Qc', 'Qz'} );
% tetramesh(yo2, yo)
