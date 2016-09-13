% Optimizing for instantaneous power loss for kinetic energy 
% Given a velocity, what is the power transfer when you tug in a given
% direction? 
% Early iterations on a plot to display this
addpath('functionsCvx','functionsHelper','dataGenerated')

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
scatter3(limitWrist(:,1),limitWrist(:,3),P)
xlabel('F_x [N]')
ylabel('T_z [Nm]')
zlabel('Power [W]')
title('Power into the system')
%%
constraints = [20 20 10e4 10e4]';
[ minP, Fnet, tensions, components ] = cvxGripMinP( A, constraints, v)


%allDataWrist = (trans(r)*allData')';
%scatter3(allDataWrist(:,1),allDataWrist(:,3),allDataWrist(:,2),60,'*')
%% %% At Wrist

constraints = [24 19.28 10e4 10e4]';
[ minP, Fnet, tensions, components ] = cvxGripMinP( A, constraints, v)

load('3DmeshLimit') % FyWrist, f, m
P = zeros(size(FyWrist));

v = [1 -4 5]';

% Calculate Power P
for ii = 1:numel(f)
    for jj = 1:numel(m); 
        P(ii,jj) = [f(ii) FyWrist(ii,jj) m(jj)]*v;        
    end
end

figure; set(gca,'fontsize',16); hold on 
% model
[X,Y] = meshgrid(f,m);
contour3(X,Y,P,30,'linewidth',2)
% Plane of zero Power
X = [-20 20 20 -20];
Y = [-0.15 -0.15 0.15 0.15];
Z = [0 0 0 0];
C = [0.5000 1.0000 1.0000 0.5000];
s = fill3(X,Y,Z,C);
alpha(s,0.3)

scatter3(Fnet(1),Fnet(3),minP,200,'o','LineWidth',4); 

xlabel('F_x [N]')
zlabel('P [W]')
ylabel('T_z [Nm]')
title('Power Dissipation')
%alpha(s,0.05) % transparency
legend('model','P = 0')


        %%
limitWrist = (trans(r)*limit')';
ATI2Wrist = trans(-d);

r = 9/2*0.0254; % Distance from object COM to object surface
d = 0.081;      % Distance from object surface to ATI
alphad = 11.35;  % [deg]

A = defineGeometry(alphad,r);

trans = @(r) [1 0 0; 0 1 0; r 0 1];

figure; set(gca,'fontsize',16); hold on 


%s = surf(f,m,FyWrist)

for nn = 1:nTrials
    numWrist = (ATI2Wrist*data{nn,2}')';
    scatter3(numWrist(:,1),numWrist(:,3),numWrist(:,2),60,'*')
end
xlabel('F_x [N]')
zlabel('F_y [N]')
ylabel('T_z [Nm]')
title('Force/Torque at Wrist')
alpha(s,0.05) % transparency
%legend(['model' data{:,1}])
legend('model', 'pulloff tests')