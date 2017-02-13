close all; clear; clc; 

%load('3DscatterLimit_paperAsymmetric.mat') % limit variable represents as accelerations about object's COM
addpath('functionsCvx','functionsHelper','dataGenerated')

limitsurfaceFile = '3DscatterLimit_AsymmetricPaper_Sept8';
load('3DscatterLimit_AsymmetricPaper_Sept8')

%% Set up Constants
% Rigid Body parameters 
ITarzz                          =  0.0128;                 % kg*m^2              Constant
mTar                            =  1.5526;                 % kg                  Constant
r                               =  0.1143;                 % m                   Constant
trans = @(rd) [1 0 0; 0 1 0; rd 0 1];

limit(isinf(limit(:,3)),:) = [];    % Get rid of erraneous vals
limit(isnan(limit(:,3)),:) = [];    % Get rid of erraneous vals
limitWrist = (trans(r)*limit')';
limitWrist=limit;

figure; set(gca,'fontsize',20); hold on;
%plotManualIsolines(limitWrist,limitWrist(:,2),'flipped')
axis tight

x = limitWrist(:,1);
z = limitWrist(:,2);  % Note I switch y and z here 
y = limitWrist(:,3);

[azimuth,elevation,r] = cart2sph(x,y,z);

fit_polar = fitSurface_polar(limitWrist,'linearinterp'); % Fitting surface w/ linear interpolation 
mag = solveSurface_polar(fit_polar,[x, y, z]);
 sfx = x.*mag; 
 sfy = y.*mag;
 sfz = z.*mag; 
[sfx, sfy, sfz] = sph2cart(azimuth, elevation,mag);

plot3(x,y,sfz,'d') 
hold on
plot3(x,y,z,'*')
hold off

% %% Fitting
% x = limitWrist(:,1);
% z = limitWrist(:,2);
% y = limitWrist(:,3);
% 
% sf = fit([x, y],z,'linearinterp')
% plot(sf,[x,y],z) 
% hold on
% plot3(x,y,z)
% 
% 
% 
% %% Polar Fitting? 
% 
% x = limitWrist(:,1);
% z = limitWrist(:,2);
% y = limitWrist(:,3);
% 
% 
% [azimuth,elevation,r] = cart2sph(x,y,z);
% 
% sf = fit([azimuth, elevation],r,'linearinterp')
% sfr = sf([azimuth, elevation]);
% [sfx, sfy, sfz] = sph2cart(azimuth, elevation,sfr);
% 
% plot3(x,y,sfz,'d') 
% hold on
% plot3(x,y,z,'*')
% hold off
% 
% tic
% sf(-1.5708,1.2189)
% toc
% 
% 
% 
% 
