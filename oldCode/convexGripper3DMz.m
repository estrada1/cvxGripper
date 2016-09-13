%% 2D Curved Surface Gripper
% Matt Estrada
% Going off Elliot's drawing in his curved gripper paper
% Doing a 2D slice to calculate result between moment/tangential force
% Super similar to convexGripperPlanar_LimitSurface_Cartesian, just
% ignoring the Fy direction
% June 29 2016

close all; clear; clc; 
addpath('functionsCvx','functionsHelper','dataGenerated')

%% Define geometry
alpha = 11.35;     % [deg]
%alpha = 20
r = 0.2286/2;      % [meter]
rd = r + .081;   % [m] Distance from COM and wrist joint
h = .009;        % [m] offset from surface

transSurf = [1 0 0 ; 0 1 0 ; r 0 1];
transATI = [1 0 0 ; 0 1 0 ; rd 0 1];

A = defineGeometry(alpha,r);

%maxAdhesion =  54.8839;

%maxAdhesion1 =  24;
%maxAdhesion2 = 19.28;
maxAdhesion1 =  20;
maxAdhesion2 = 20;
maxAdhesion = max([maxAdhesion1 maxAdhesion2]); 
constraints = [maxAdhesion1; maxAdhesion2; 1000000; 1000000];

% Double check how high we should sweep
[ beta, unit_vect, components ] = ...
    cvxGripBeta( alpha, r, [0;1;0], constraints);
maxFy = beta

limit = []; 

for Fy = 0:1:maxFy
%for Fy = 0:2.5:maxFy   
    
    Fx = -maxAdhesion:2:maxAdhesion;
    
    objective = 'max';
    Mz_max = zeros(size(Fx));
    for nn = 1:numel(Fx)
        
        fy = Fy;
        fx = Fx(nn);
        [Mz, vect] = cvxGripMz(A, fx, fy, constraints, objective);
        Mz_max(nn) = Mz;
        disp([ 'Trial ' num2str(nn) ' of ' num2str(numel(Fx))])

    end

    objective = 'min';
    Mz_min = zeros(size(Fx));

    for nn = 1:numel(Fx)

        fx = Fx(nn);
        [Mz, vect] = cvxGripMz(A, fx, fy, constraints, objective);
        Mz_min(nn) = Mz;
        disp([ 'Trial ' num2str(nn) ' of ' num2str(numel(Fx))])

    end
    
    thisLimit = [Fx' Fy*ones(length(Fx),1) Mz_max'; Fx' Fy*ones(length(Fx),1) Mz_min'];
    limit = [limit; thisLimit];
end

%%
figure
scatter3(limit(:,1),limit(:,3),limit(:,2))

%% Translate this stuff
% FxFyMzWrist = transSurf*limit';
% FxFyMzWrist = FxFyMzWrist';
% 
% %load('2016_07_07_trials1_data')
% nonumbers = isnan(FxFyMzWrist(:,1));
% FxFyMzWrist(nonumbers,:) = [];
% 
% xlim = 10; 
% ylim = 7; 
% zlim = 0.5;
% [wristx, wristy, wristz] = meshgrid(-xlim:xlim,-ylim:0,-zlim:0.1:zlim)
% wrist = [reshape(wristx,[numel(wristx) 1]) reshape(wristy,[numel(wristx) 1]) reshape(wristz,[numel(wristx) 1])]
% 
% figure
% set(gca,'fontsize',16)
% hold on
% 
% Fy = FxFyMzWrist(:,2);
% Fy_unit = abs(Fy)/max(abs(Fy))/10
% fyn = length(FxFyMzWrist(:,2)); 
% scatter3(FxFyMzWrist(:,1),FxFyMzWrist(:,2),FxFyMzWrist(:,3),10,[ Fy_unit Fy_unit ones(fyn,1)])
% scatter3(wrist(:,1),wrist(:,2),wrist(:,3),10)
% %legend('Adhesive Capabilities')
% %legend('Example of Wrist Compliance Space')
% 
% xlabel('F_x [N]')
% ylabel('F_y [N]')
% zlabel('T_z [Nm]')
% title('Sustainable Force/Torque at Wrist')

parameters = struct('alpha',alpha,'r',r,'A',A, 'constraints',constraints);
save('dataGenerated/3DscatterLimit_Sept7','limit','parameters');
