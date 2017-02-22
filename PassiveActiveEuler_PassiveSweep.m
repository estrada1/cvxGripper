
close all; clear; clc; 
addpath('functionsCvx','functionsHelper','dataGenerated')

MODE = 'SINGLE';
% MODE = 'SWEEP';
% MODE = 'SWEEPDYNAMICS'

%% Define object properties 
Tar.ITarzz                          =  0.0128;                 % kg*m^2              Constant
Tar.mTar                            =  1.5526;                 % kg                  Constant
Tar.r                               =  0.1143;                 % m                   Constant
massMatrix = [Tar.mTar 0 0; 0 Tar.mTar 0; 0 0 Tar.ITarzz]; 

%% Gripper Properties
r = Tar.r; % Distance from object COM to object surface
alphad = 11.35;  % [deg]
A = defineGeometry(alphad,r);
trans = @(r) [1 0 0; 0 1 0; r 0 1];
Awrist = trans(r)*A; 

% Adhesive limits
limits = [24.0 19.28]; 

%% Passive or Active gripper 

%PASSIVE_OR_ACTIVE = 'PASSIVE';
PASSIVE_OR_ACTIVE = 'ACTIVE';
%PASSIVE_OR_ACTIVE = 'ONE_DOF' 
trialName = 'PaperRevisions'; 
limitsurfaceFile = '3DscatterLimit_AsymmetricPaper_Sept8';


%% Parameters for Passive or Active 


%% Object's initial conditions 
r = Tar.r; 
%q0 = [0 -r 0 .25 -0.25 .3]' % Set initial velocity with q0
%q0 = [0 -r 0 -0.25 -0.25 2*pi]' %< --- Good comparison , sort of quick-stop
q0 = [0 -r 0 0 -0.2 2*pi]' %< --- Good comparison, used in paper submission
%q0 = [0 -r 0 0 -0.2 3*pi]' %< --- Good comparison, used in paper draft
%q0 = [0 -r 0 .3 -1 4*pi]'; <--- Good settings for aggressive active control

%% Setup simulation 
switch MODE
    case 'SINGLE'
        printouts = 1; 
        tuning = [15 .3]; % = [wn zeta
        tuning = [0.5 .6];
        tuning = [.6 .6];
        tuning = [.5 .8];
            [Q,U,K,QTarB,t,limitWrist,tensions,success] = PassiveActiveEuler(Tar,q0,...
                PASSIVE_OR_ACTIVE,tuning,trialName,Awrist,limits,limitsurfaceFile,printouts);
            %%
        PassiveActiveEuler_plot(Q,U,K,QTarB,t,limitWrist,tensions,limits)
        
    case 'SWEEP'
        % Simulation
        printouts = 0; 
%         wn = 3:3:15;
%         zeta = 0.3:0.3:1.2;
        wn = .2:.1:1.4;
        zeta = 0.3:0.1:1.2;

        [ww,zz] = meshgrid(wn,zeta); 
        ww = reshape(ww,[numel(ww),1]); 
        zz = reshape(zz,[numel(zz),1]);

        trials = [ww zz] ;
        %%
        % trials = [12 .4; ...
        %             5 1;...
        %             12 1];
        nTrials = length(trials); 
        results = zeros(nTrials,4); 

        for kk = 1:nTrials
            tuning = trials(kk,:); 
            [Q,U,K,QTarB,t,limitWrist,tensions,success] = PassiveActiveEuler(Tar,q0,...
                PASSIVE_OR_ACTIVE,tuning,trialName,Awrist,limits,limitsurfaceFile,printouts);
            results(kk,:) = [tuning success t(end)]
        end
        
        %% Results 
        outcome = results(:,3)==1; 
        indSuccess = find(results(:,3)==1);
        indFail = find(results(:,3)==0);
        
        figure; set(gca,'fontsize',20); hold on;
        scatter(results(indSuccess,1),results(indSuccess,2),results(indSuccess,4)*20);
        scatter(results(indFail,1),results(indFail,2),'rx'); hold on;
        axis([0 1.5 0 1.25])
        ylabel('\zeta');
        xlabel('\omega_{n}'); 

        figure; set(gca,'fontsize',20); hold on;
        scatter3(results(indSuccess,1),results(indSuccess,2),results(indSuccess,4));
        scatter3(results(indFail,1),results(indFail,2),zeros(length(indFail),1),'rx'); hold on;
        axis([0 1.5 0 1.25])
        ylabel('\zeta');
        xlabel('\omega_{n}'); 
        zlabel('Settling time [sec]')
        grid on;
        
        
    case 'SWEEPDYNAMICS'
        % Simulation
        printouts = 0; 

        tuning = [.5 .8];

        trials = .4:.1:2;

        nTrials = length(trials); 
        results = zeros(nTrials,3); 
        
        for kk = 1:nTrials
            mag =  trials(kk); 
            [Q,U,K,QTarB,t,limitWrist,tensions,success] = PassiveActiveEuler(Tar,mag*q0,...
                'ACTIVE',tuning,trialName,Awrist,limits,limitsurfaceFile,printouts);
            results_ACTIVE(kk,:) = [trials(kk) success t(end)]
        end
        
        for kk = 1:nTrials
            mag =  trials(kk); 
            [Q,U,K,QTarB,t,limitWrist,tensions,success] = PassiveActiveEuler(Tar,mag*q0,...
                'PASSIVE',tuning,trialName,Awrist,limits,limitsurfaceFile,printouts);
            results_PASSIVE(kk,:) = [trials(kk) success t(end)]
        end
        

        
        save('FirstDynamicSweep','results_ACTIVE','results_PASSIVE')
        
        %% Results 
        
        outcome = results(:,3)==1; 
        indSuccess = find(results(:,3)==1);
        indFail = find(results(:,3)==0);
        
        Knominal = 1/2*q0(4:6)'*massMatrix*q0(4:6); 
        
        figure; set(gca,'fontsize',20); hold on;
        log(results_PASSIVE(:,1).^2*Knominal,results_PASSIVE(:,3),'*');
        log(results_ACTIVE(:,1).^2*Knominal,results_ACTIVE(:,3),'*');
        %scatter(results(indFail,1),results(indFail,2),'rx'); hold on;
        ylabel('t_{settle}');
        xlabel('K [J]'); 
        legend('Passive','Active')
        
        


end

%% Plot

%tfinal = find(K>.0001,1,'last')



%% Calculate 


% nSim = length(U); 
% tensions = zeros(4,nSim);
% 
% for kk = 1:nSim
%     tensions(:,kk) = ( lsqnonneg(Awrist,U(:,kk)) )';
% end





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

