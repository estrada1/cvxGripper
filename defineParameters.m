% defineParameters.m
% Matt Estrada
% Sept 8 2016
% Set up gripper geometry and adhesion capabilities 
% Streamline the code I write at the top of every script in cvxGripper

addpath('functionsCvx','functionsHelper','dataGenerated')
trans = @(rd) [1 0 0; 0 1 0; rd 0 1];

% %% Free Flyer setup 
% % Define Geometry 
% alphad = 11.35;      % [deg]
% r = 9/2*0.0254;     % [m]
% Acm = defineGeometry(alphad,r);
% Awrist = trans(r)*Acm; 
% 
% % Define Adhesion 
% maxAdhesion1 =  24;
% maxAdhesion2 = 19.28;
% maxAdhesion = max([maxAdhesion1 maxAdhesion2]); 
% constraints = [maxAdhesion1; maxAdhesion2; 1000000; 1000000];

% %% Generic Numbers
% % Define Geometry 
% alphad = 20;      % [deg]
% r = 0.1;     % [m]
% Acm = defineGeometry(alphad,r);
% Awrist = trans(r)*Acm; 
% 
% % Define Adhesion 
% maxAdhesion1 =  20;
% maxAdhesion2 = 20;
% maxAdhesion = max([maxAdhesion1 maxAdhesion2]); 
% constraints = [maxAdhesion1; maxAdhesion2; 1000000; 1000000];

%% Roshena Numbers
% Define Geometry 
alphad = 15;      % [deg]
r = 1.2;     % [m]
Acm = defineGeometry(alphad,r);
Awrist = trans(r)*Acm; 

% Define Adhesion 
maxAdhesion1 =  60;
maxAdhesion2 = 60;
maxAdhesion = max([maxAdhesion1 maxAdhesion2]); 
constraints = [maxAdhesion1; maxAdhesion2; 1000000; 1000000];


parameters = struct('alphad',alphad,'r',r,'Acm',Acm, 'constraints',constraints);


