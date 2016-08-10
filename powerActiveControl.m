% Optimizing for instantaneous power loss for kinetic energy 

%% Simple example
scale = 1/10;
Fx = 0:scale/100:scale; 
Fy = 1*(1-Fx/scale);
F = [Fx' Fy'];
v = [1 1]';
P = F*v; 

figure
plot(Fx,Fy,'-',Fx,P,'--','linewidth',3)
legend('Force','Power')
%% Sweeping over 
load('3DscatterLimit4')
%%
r = 9/2*0.0254; % Distance from object COM to object surface
d = 0.081;      % Distance from object surface to ATI
alphad = 11.35;  % [deg]

A = defineGeometry(alphad,r);

trans = @(r) [1 0 0; 0 1 0; r 0 1];

%%
nonumbers = isinf(limit(:,3)); 
limit(nonumbers,:) = []; 
limitWrist = (trans(r)*limit')';

v = [1 -4 5]';
P = limitWrist*v; 

figure; set(gca,'fontsize',16); hold on 
scatter3(limitWrist(:,1),limitWrist(:,3),limitWrist(:,2),10)
xlabel('F_x [N]')
ylabel('T_z [Nm]')
zlabel('F_Y [N]')
title('Force/Torque at Contact')

figure; set(gca,'fontsize',16); hold on 
scatter3(limitWrist(:,1),limitWrist(:,3),P,10)
xlabel('F_x [N]')
ylabel('T_z [Nm]')
zlabel('Power [W]')
title('Power into the system')
%%
constraints = [20 20 10e4 10e4]';
[ minP, Fnet, tensions, components ] = cvxGripMinP( A, constraints, v)