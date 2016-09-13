%% 2D Curved Surface Gripper
% Matt Estrada
% Generate limit surface for all 3 DOF, fx fy mz
% Using method where we constrain fx / fy anc calculate Mz
% Sept 8, 2016

%% Set up parameters 

close all; clear; clc; 
dataName = 'dataGenerated/3DscatterLimit_AsymmetricPaper_Sept8'; 

% Define geometry and adhesion capabilities 
defineParameters; % A, alpha, maxAdhesion 

A = Awrist; 
maxFy = cvxGripMaxFy( A, constraints)

%% Sweep space 
limit = []; 

for fy = 0:1:maxFy
    
    [thisLimit, tensions] = limitSurfaceMz2D(A,constraints, fy);
    
    limit = [limit; thisLimit];
end

% %% Special case
% for fy = 0:1:maxFy
%     
%     [thisLimit, tensions] = limitSurfaceMz2D(A,constraints, fy);
%     
%     limit = [limit; thisLimit];
% end
%% Plot fig just to double check 
figure
scatter3(limit(:,1),limit(:,3),limit(:,2))

%% Save file 
save(dataName,'limit','parameters');
