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
y = limitWrist(:,2);  % Note I switch y and z here 
z = limitWrist(:,3);

[azimuth,elevation,r] = cart2sph(x,y,z);

fit_polar = fitSurface_polar(limitWrist,'linearinterp'); % Fitting surface w/ linear interpolation 
mag = solveSurface_polar(fit_polar,[x, y, z]);
%  sfx = x.*mag; 
%  sfy = y.*mag;
%  sfz = z.*mag; 
[sfx, sfy, sfz] = sph2cart(azimuth, elevation,mag);

test1 = [0 0 1;...
        0 1 0;...
        1 0 0;...
        0 0 1;];
        %0 8 0];
[mag1, comp1] = solveSurface_polar(fit_polar,test1)

theta = 0:.01:pi;

test2 = [zeros(length(theta),1) , sin(theta)', cos(theta)'];
[mag2, comp2] = solveSurface_polar(fit_polar,test2)

plot3(sfx,sfy,sfz,'d') 
hold on
plot3(x,y,z,'*')
%plot3(test1(:,1),test1(:,2),test1(:,3),'s','MarkerSize',8);
plot3(comp1(1,:),comp1(2,:),comp1(3,:),'gs','MarkerSize',15);
plot3(comp2(1,:),comp2(2,:),comp2(3,:),'ks','MarkerSize',15);

plotManualIsolines(limitWrist,limitWrist(:,2));
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
