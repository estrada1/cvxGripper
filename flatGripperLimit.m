%% Plotting curve surface limit data

% This is a mesh table that has the first row/column as 0-180
limit3D = csvread('data/3D Limit Curve_every10deg.csv');

%% Do we just want the middle ROW?
i = 11;
limit3D(i,1);
limit2D = limit3D(i,2:end);
phi = limit3D(1,2:end);
fx = limit2D.*-cosd(phi);
fy = limit2D.*-sind(phi);

figure; set(gca,'fontsize',16); hold on 
plot(fx,fy,'*')
axis equal
title('Sweeping Phi')

%% % Do we just want the middle COLUMN?
 
limit3D(1,i);
limit2D = limit3D(2:end,i);
theta = limit3D(2:end,1);
fx = limit2D.*-cosd(theta);
fy = limit2D.*-sind(theta);

figure; set(gca,'fontsize',16); hold on 
plot(fx,fy,'*')
axis equal
title('Sweeping Theta')