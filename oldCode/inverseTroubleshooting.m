%% Trying to deduce why the inverse matrix of A isn't giving me expect values for adhesion load

load('3DscatterLimit3.mat')

nonumbers = isnan(limit(:,1));
limit(nonumbers,:) = [];
noinf = isinf(limit(:,3)); 
limit(noinf,:) = [];

figure
scatter3(limit(:,1),limit(:,3),limit(:,2))
title('Accelerations')


%% Define some stuff 
r = 9/2*0.0254; % Distance from COM to back
d = 0.081;      % Distance from surface to ATI
alpha = 11.35;  % [deg]

A = defineGeometry(alpha,r);
trans = @(r) [1 0 0; 0 1 0; r 0 1];

%% Double check on straight-up accelerations

nSim = length(limit);

tensions = zeros(nSim,4);

for kk = 1:nSim
    tensions(kk,:) = ( lsqnonneg(A,limit(kk,:)') )';
end
tensions

% This seems to check out 

%% Translate to Wrist and verify this still works

trans_wrist = trans(r);
limit_wrist = (trans_wrist*limit')'
figure
figure
scatter3(limit_wrist(:,1),limit_wrist(:,2),limit_wrist(:,3))
title('Accelerations at Wrist')

% Plot wrist limits 

    hold on
    ATIoffset = -0.081;
    twisting = importATI('data/2016_07_08_trial2/twisting',ATIoffset)
    scatter3(twisting(:,1),twisting(:,2),twisting(:,3),60,'*')

    pull = importATI('data/2016_07_08_trial2/Zdir',ATIoffset)
    scatter3(pull(:,1),pull(:,2),pull(:,3),60,'*')

    tangential = importATI('data/2016_07_08_trial2/Ydir',ATIoffset)
    scatter3(tangential(:,1),tangential(:,2),tangential(:,3),60,'*')

    legend('predicted','twisting','Normal','Tangential')

% Calc the presumed tensions for simulation
nSim = length(limit_wrist);
tensions = zeros(nSim,4);

for kk = 1:nSim
    tensions(kk,:) = ( lsqnonneg(trans_wrist*A,limit_wrist(kk,:)') )';
end
tensions; % Checks out

% Calc presumed tensions for data
data = [twisting;pull];
nData = length(data);
for kk = 1:nData
    tensionsData(kk,:) = ( lsqnonneg(trans_wrist*A,data(kk,:)') )';
end
tensionsData

%% Translate to ATI and verify it still works
trans_ATI = trans(r+d);
limit_ATI = (trans_ATI*limit')'
figure
scatter3(limit_ATI(:,1),limit_ATI(:,3),limit_ATI(:,2))
title('Forces at ATI')
xlabel('F_x [N]')
ylabel('F_y [N]')
zlabel('T_z [Nm]')
title('Sustainable Force/Torque at ATI')


nSim = length(limit_ATI);
tensions = zeros(nSim,4);

for kk = 1:nSim
    tensions(kk,:) = ( lsqnonneg(trans_ATI*A,limit_ATI(kk,:)') )';
end
tensions

% Checks out

%% Conclusion
% So it looks like I was just getting compression/tension mixed up here.
% The Fy force was being registered as positive in the ATI sensor (makes
% sense) but I was calculating negative Fy forces for my model's limit 
