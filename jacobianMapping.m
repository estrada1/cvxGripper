% Matt Estrada
% Aug 28 2016 
% Mapping gripper limitations to joint torque limitations for an
% articulated arm. 
% Using rvctool from Peter Corke (downloaded his toolbox) for computing the
% jacobian

close all; clear; 

% Define DH Parameters
syms l
subs(l,1)
l = 1
L(1) = Revolute('d', 0, 'a', l, 'alpha', 0);
L(2) = Revolute('d', 0, 'a', l, 'alpha', 0);
L(3) = Revolute('d', 0, 'a', 0, 'alpha', 0);


% Create SerialLink Object
twolink = SerialLink(L, 'name', 'two link');

% Pose (joint angles)
q = [pi/6 pi/2 -pi/4];

j0 = twolink.jacobn(q)

twolink.plot(q)

% Only need this 2D jacobian
% Must rotate matrix to make Corke's axese align with mine
% Rotate +90 deg in +z direction 
J = j0([1 2 6],:)

load('3DscatterLimit_paperAsymmetric.mat');
r = 9/2*0.0254; % Distance from object COM to object surface
trans = @(r) [1 0 0; 0 1 0; r 0 1];
limit(isinf(limit(:,3)),:) = [];% Get rid of erraneous vals
limitWrist = (trans(r)*limit')';
th = -pi/2; 
rotth = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];

Q = (J'*rotth*limitWrist')'

plotManualIsolines(Q,limitWrist(:,2))
ax = [max(Q(:,1)) 0 0; min(Q(:,1)) 0 0];
ay = [0 max(Q(:,2)) 0; 0 min(Q(:,2)) 0];
az = [0 0 max(Q(:,3)); 0 0 min(Q(:,3))];

plot3(ax(:,1),ax(:,3),ax(:,2),'k','LineWidth',4)
plot3(ay(:,1),ay(:,3),ay(:,2),'k','LineWidth',4)
plot3(az(:,1),az(:,3),az(:,2),'k','LineWidth',4)

scatter3(0,0,0,'ro')
xlabel('\tau_1')
ylabel('\tau_3')
zlabel('\tau_2')
title('Sustainable joint torques')

axis tight
% figure; set(gca,'fontsize',16); hold on;
% scatter3(Q(:,1),Q(:,2),Q(:,3),10) % Predicted Limit
% scatter3(0,0,0,'ro')
% xlabel('\tau_1')
% ylabel('\tau_2')
% zlabel('\tau_3')
% title('Sustainable joint torques')


