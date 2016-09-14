%% Plot data in a more inuitive form-- connecting isolines. 
% Matt Estrada
% Aug 30 2016
% Tried plotting isolines already but it didn't connect the ends, which
% sucked 
close all; clear; clc; 
addpath('functionsCvx','functionsHelper','dataGenerated')

r = 9/2*0.0254; % Distance from object COM to object surface
d = 0.081;      % Distance from object surface to ATI
trans = @(r) [1 0 0; 0 1 0; r 0 1];

% Load in simulated limit surface
%load('dataGenerated/3DscatterLimit_AsymmetricPaper_Sept8') % limit variable represents as accelerations about object's COM
load('3DscatterLimit_Adhesive20_Sept13')
limit(isinf(limit(:,3)),:) = [];% Get rid of erraneous vals
limit(isnan(limit(:,3)),:) = [];% Get rid of erraneous vals
%limitWrist = (trans(r)*limit')';
limitWrist=limit;

figure; set(gca,'fontsize',20); hold on;
plotManualIsolines(limitWrist,limitWrist(:,2))
axis tight

figure;
subplot(2,1,1)
set(gca,'fontsize',16); hold on;
plotManualIsolines(limitWrist,limitWrist(:,2),'flipped')
axis tight
subplot(2,1,2)
set(gca,'fontsize',16); hold on;
plotManualIsolines(limitWrist,limitWrist(:,2))
axis tight

