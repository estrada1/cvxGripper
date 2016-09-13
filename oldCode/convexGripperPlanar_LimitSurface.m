%% 3D Curved Surface Gripper
% Going off Elliot's drawing in his curved gripper paper
% Matt Estrada
% June 23 2016
% Aug 11 Description Update: 
% This was an early sweep, tugging in a "dome" of directions
% Solving for Mz given a Fx Fy was a clearer way to display this data
addpath('functionsCvx','functionsHelper','dataGenerated')

sweep_phi = -90:5:90; 
%sweep_theta = 0:5:180; % Rotation about Nx
sweep_theta = 0; % Rotation about Nx

% Rotation Matrices along x and y axis, with input in degrees
Rxd = @(th) [1 0 0 ; 0 cosd(th) sind(th); 0 -sind(th) cosd(th)]; 
Ryd = @(th) [cosd(th) 0 -sind(th); 0 1 0 ; sind(th) 0 cosd(th)]; 


limit = zeros(length(sweep_theta),1); 
tension = zeros(length(sweep_theta),4); 
limit3 = zeros(length(sweep_theta),3);
alpha = 15;     % [deg]
theta = 90;     
r = 0.100;      % [meter]
t1 = [cosd(alpha); sind(alpha); - r];
t2 = [-cosd(alpha); sind(alpha); r];
c1 = [sind(alpha); -cosd(alpha); 0];
c2 = [-sind(alpha); -cosd(alpha); 0];

A =  [t1 t2 c1 c2];

nz = [ 0 0 1]'; 

directions = zeros(length(sweep_theta)^2, 3);
tension = zeros(length(sweep_theta)^2, 4);
limit = zeros(length(sweep_theta)^2, 1);
vectorNoFy = zeros(length(sweep_theta)^2, 3); 


for nn = 1:length(sweep_theta)

    theta = sweep_theta(nn); 
    Rtheta = Rxd(theta);
    
    for kk = 1:length(sweep_phi)
        
        phi = sweep_phi(kk); 
        Rphi = Ryd(phi); 
        d = Rphi*Rtheta*nz; 
        
        index = length(sweep_theta)*(nn-1) + kk;
        directions(index, :) = d; 
        
        cvx_begin

            variable x(4,1)
            variable fnet(1,1)

            maximize( fnet )

            subject to

                x<=[10; 10; 10000; 10000]
                x>=[0; 0; 0; 0]
                fnet*d ==  A * x

        cvx_end
    
       limit(index) = cvx_optval ;
       tension(index,:) = x;
       vectorNoFy(index,:) = (A*x)'; 
    end
   
end

% closeForm = 2*x(1)*sind(alpha)^2/(sind(alpha+theta))
% 
% limit
% %tension

%% 
load('convexGripperPlanar_LimitSurface_1369pts.mat')

close all 

figure
set(gca,'fontsize',16)
hold on
scatter3(directions(:,1),directions(:,2),directions(:,3))
title('Directions Tested')
xlabel('Fx')
ylabel('Fy')
zlabel('Mz')


figure
set(gca,'fontsize',16)
hold on
scatter3(vectorNoFy(:,1),vectorNoFy(:,2),vectorNoFy(:,3))
title('Directions Tested')
xlabel('Fx')
ylabel('Fy')
zlabel('Mz')

%% Double check / visualize
load('convexGripperPlanar_LimitSurface_1369pts.mat')

% Take planar slice 
vectorNoFy = vector(directions(:,2) == 0,:);
vectorNoMz = vector(directions(:,3) == 0,:);

% No normal force
figure
set(gca,'fontsize',16)
hold on
M = vectorNoFy(:,[1 3])'; 
T = '-';
fill(M(1,:),M(2,:),'g')
plotv(M,T)
title('Tradeoff between Tangential Force and Moment')
plot(M(1,:),M(2,:),'*')
xlabel('Fx')
ylabel('Mz')

% No moment

figure
set(gca,'fontsize',16)
hold on
M = vectorNoMz(:,[1 2])'; 
T = '-';
fill(M(1,:),M(2,:),'g')
plotv(M,T)
title('Tradeoff between Normal and Tangential Force')
xlabel('Fx')
ylabel('Fy')


%% 3D Quiver Plot 
load('convexGripperPlanar_LimitSurface_1369pts.mat')

zed = zeros(length(vector),1);

figure
set(gca,'fontsize',16)
hold on

quiver3(zed,zed,zed,vector(:,1),vector(:,2),vector(:,3))
scatter3(vector(:,1),vector(:,2),vector(:,3))
title('Curved Gripper Limit Surface from FBD')
xlabel('ax [m/s^2]')
ylabel('ay [m/s^2]')
zlabel('alphaz [rad/sec^2]')

% Interesting cases
index_p00 = 1;      % move without rotating
index_00p = 19;     % torque only
index_p0p = 10;     % counteracting torque/force
index_p0n = 1368;   % torque/force add
interest = [index_p00; index_00p; index_p0p; index_p0n];


scatter3(vector(interest,1),vector(interest,2),vector(interest,3),200*ones(length(interest),1),'k*')

    
%% Try translating to pin joint
trans = -[1 0 0 ; 0 1 0 ; r 0 1];
vector_surf= trans*vector';
vector_surf = vector_surf';

figure
set(gca,'fontsize',16)
hold on

quiver3(zed,zed,zed,vector_surf(:,1),vector_surf(:,2),vector_surf(:,3))
scatter3(vector_surf(:,1),vector_surf(:,2),vector_surf(:,3))
title('Forces felt at Wrist')
xlabel('Fx [N]')
ylabel('Fy [N]')
zlabel('Tz [Nm]')

scatter3(vector_surf(interest,1),vector_surf(interest,2),vector_surf(interest,3),200*ones(length(interest),1),'k*')



%% Plot the forces as 3D vectors
figure
zed2 = zeros(4,1)';
quiver3(zed2,zed2,zed2,A(1,:),A(2,:),A(3,:))

%% Points for a Convex Hull
% This does not look promising 
B = de2bi([1:15])

p = A*B'
K = convhull(p(1,:),p(2,:),p(3,:))

figure
trisurf(K,p(1,:),p(2,:),p(3,:))
title('calculated')
xlabel('Fx [N]')
ylabel('Fy [N]')
zlabel('Tz [Nm]')

p2 = vector'
K2 = convhull(p2(1,:),p2(2,:),p2(3,:))


figure
trisurf(K2,p2(1,:),p2(2,:),p2(3,:))
title('optimized')
xlabel('Fx [N]')
ylabel('Fy [N]')
zlabel('Tz [Nm]')





