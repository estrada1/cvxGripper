close all; clear; clc;
addpath('functionsCvx','functionsHelper','dataGenerated')

open('success_ATI.fig')

% Force vs. time
subplot(2,1,1)
set(gca,'fontsize',16); hold on;

% Force space
subplot(2,1,2)
set(gca,'fontsize',16); hold on;
axis([-20 20 -2 7 -0.4 0.3])
set(gca,'fontsize',16); hold on;

load('3DscatterLimit_paperAsymmetric')
r = 9/2*0.0254; % Distance from object COM to object surface
limit(isinf(limit(:,3)),:) = [];% Get rid of erraneous vals
%limitWrist = (trans(r)*limit')';
limitWrist=limit;
trans = @(r) [1 0 0; 0 1 0; r 0 1];
limitWrist = (trans(r)*limit')';

plotManualIsolines(limitWrist,limitWrist(:,2))
xlabel('F_x')
ylabel('F_y')
zlabel('M_z')
legend('Force at gripper','Limit Surface')

%scatter3(