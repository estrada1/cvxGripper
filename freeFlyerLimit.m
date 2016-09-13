%% Calculating Free Flyer limitations
% Another application of limit surface envelope 
syms mt It rt real;
syms mB IB rB real; 
syms ms Icm rcm real; 


ms = mB+mt; 
rcm = (mt*rt-rB*mB)/(mt+mB);
% Target's inertia is too damn complicated to display
%Icm = simplify(It + IB + (rB+rcm)^2*mB + (rt+rcm)^2*mt)

B = [1 0 0; 0 1 0; 0 rB 1];

iC = [1 0 -rcm; 0 1 0; 0 0 1];
C = inv(iC)
D = [0; -rcm; 0];
Ms = [ms 0 0; 0 ms 0; 0 0 Icm];
Mt = [mt 0 0; 0 mt 0; 0 0 It];

% Mapping Forces
simplify(B*Ms*C*inv(Mt))

% Mapping centrifugal
simplify(B*Ms*C*D)

%% Define values
mt = 1; 
rt = 1;
It = 1; 

mB = 1;
rB = 1; 
IB = 1; 
Icm = subs(It + IB + (rB+rcm)^2*mB + (rt+rcm)^2*mt);

AA = subs(B*Ms*C*inv(Mt))

load('3DscatterLimit_paperAsymmetric.mat');
r = 9/2*0.0254; % Distance from object COM to object surface
trans = @(r) [1 0 0; 0 1 0; r 0 1];
limit(isinf(limit(:,3)),:) = []; % Get rid of erraneous vals
limitWrist = (trans(r)*limit')';

limitFF = (AA*limitWrist')'

plotManualIsolines(limitFF,limitWrist(:,2))
ax = [max(limitFF(:,1)) 0 0; min(limitFF(:,1)) 0 0];
ay = [0 max(limitFF(:,2)) 0; 0 min(limitFF(:,2)) 0];
az = [0 0 max(limitFF(:,3)); 0 0 min(limitFF(:,3))];

plot3(ax(:,1),ax(:,3),ax(:,2),'k','LineWidth',4)
plot3(ay(:,1),ay(:,3),ay(:,2),'k','LineWidth',4)
plot3(az(:,1),az(:,3),az(:,2),'k','LineWidth',4)

scatter3(0,0,0,'ro')
xlabel('thruster_x')
ylabel('\tau_{RW}')
zlabel('thruster_y')
title('Sustainable Free Flyer Actuation')

axis tight
