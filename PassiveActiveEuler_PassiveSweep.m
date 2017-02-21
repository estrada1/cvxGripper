
close all; clear; clc; 
addpath('functionsCvx','functionsHelper','dataGenerated')

%% Define object properties 
Tar.ITarzz                          =  0.0128;                 % kg*m^2              Constant
Tar.mTar                            =  1.5526;                 % kg                  Constant
Tar.r                               =  0.1143;                 % m                   Constant

%% Passive or Active gripper 
% 
PASSIVE_OR_ACTIVE = 'PASSIVE';
%PASSIVE_OR_ACTIVE = 'ACTIVE';
%PASSIVE_OR_ACTIVE = 'ONE_DOF' 
trialName = 'NeginAdapt_Oct10_MoreAngularMomentum'; 

limitsurfaceFile = '3DscatterLimit_AsymmetricPaper_Sept8';

%% Object's initial conditions 
r = Tar.r; 
%q0 = [0 -r 0 .25 -0.25 .3]' % Set initial velocity with q0
%q0 = [0 -r 0 -0.25 -0.25 2*pi]' %< --- Good comparison , sort of quick-stop
q0 = [0 -r 0 0 -0.2 2*pi]' %< --- Good comparison, used in paper submission
%q0 = [0 -r 0 0 -0.2 3*pi]' %< --- Good comparison, used in paper draft
%q0 = [0 -r 0 .3 -1 4*pi]'; <--- Good settings for aggressive active control


%% Simulation
printouts = 0; 
[Q,U,K,QTarB,t,limitWrist,success] = PassiveActiveEuler(Tar,q0,PASSIVE_OR_ACTIVE,trialName,limitsurfaceFile,printouts);

%% Plot
PassiveActiveEuler_plot(Q,U,K,QTarB,t,limitWrist)
%tfinal = find(K>.0001,1,'last')

%% Calculate 
r = Tar.r; % Distance from object COM to object surface
alphad = 11.35;  % [deg]
A = defineGeometry(alphad,r);
trans = @(r) [1 0 0; 0 1 0; r 0 1];
Awrist = trans(r)*A; 

% Adhesive limits
limit1 = 24.0; 
limit2 = 19.28; 

nSim = length(U); 
tensions = zeros(4,nSim);

for kk = 1:nSim
    tensions(:,kk) = ( lsqnonneg(Awrist,U(:,kk)) )';
end

lm1 = limit1*ones(1,length(t)); 
lm2 = limit2*ones(1,length(t));

figure; set(gca,'fontsize',16); hold on 
plot(t,lm1,'m--',t,lm2,'b--',t,tensions(1,:),'m',t,tensions(2,:),'b','LineWidth',2); 
legend('limit1','limit2','adhesive1','adhesive2');
title('Adhesive Loading');
ylabel('Force [N]'); 
xlabel('time [s]')

% %% Sweep for settling time 
% sweep = [1/sqrt(2) 1 sqrt(2) 2]; 
% for kk = 1:length(sweep)
%     mag = sweep(kk); 
%     [Q,U,K,QTarB,t,limitWrist,success] = PassiveActiveEuler(Tar,q0*mag,PASSIVE_OR_ACTIVE,trialName,limitsurfaceFile,printouts);
%     tfinal_PASSIVE(kk) = (t(end)); 
%     
%     [Q,U,K,QTarB,t,limitWrist,success] = PassiveActiveEuler(Tar,q0*mag,'ACTIVE',trialName,limitsurfaceFile,printouts);
%     tfinal_ACTIVE(kk) = (t(end)); 
% end
%     %%
%     
% figure
% plot(sweep.^2,tfinal_PASSIVE,'*','MarkerSize',15); hold on; 
% plot(sweep.^2,tfinal_ACTIVE,'*','MarkerSize',15);  
% % axis([0 length(tfinal) 0 max(tfinal)])

