%% Trying to compare predicted limit surface and data
% Matt Estrada
% July 10 2016
clear
close all
clc
addpath('functionsCvx','functionsHelper','dataGenerated')

r = 9/2*0.0254; % Distance from object COM to object surface
d = 0.081;      % Distance from object surface to ATI
alphad = 11.35;  % [deg]

A = defineGeometry(alphad,r);

trans = @(r) [1 0 0; 0 1 0; r 0 1];

% Load in simulated limit surface
load('3DscatterLimit_paperAsymmetric.mat') % limit variable represents as accelerations about object's COM
limitATI = (trans(r+d)*limit')';

% July 22/23 Trials
%load('2016_07_22_Pulloff')  % angled pull 
%load('2016_07_27_PaperPulloff')  % angled pull 
%load('2016_07_27_PaperPulloff_Aug10')
load('2016_09_11_PaperPulloff')
nTrials = length(data); 

figure; set(gca,'fontsize',16); hold on 

scatter3(limitATI(:,1),limitATI(:,2),limitATI(:,3),10)

for nn = 1:nTrials
    num = data{nn,2};
    scatter3(num(:,1),num(:,2),num(:,3),60,'*')
end

xlabel('F_x [N]')
ylabel('F_y [N]')
zlabel('T_z [Nm]')
title('Force/Torque at ATI')
legend(['model' data{:,1}])

%% At Wrist
limitWrist = (trans(r)*limit')';
ATI2Wrist = trans(-d);
figure; set(gca,'fontsize',16); hold on 

scatter3(limitWrist(:,1),limitWrist(:,2),limitWrist(:,3),10)

for nn = 1:nTrials
    numWrist = (ATI2Wrist*data{nn,2}')';
    scatter3(numWrist(:,1),numWrist(:,2),numWrist(:,3),60,'*')
end

xlabel('F_x [N]')
ylabel('F_y [N]')
zlabel('T_z [Nm]')
title('Force/Torque at Wrist')
legend(['model' data{:,1}])


%% Fit data to an adhesive limit
allData = []; 
tensions = zeros(nTrials,4)

for jj = 1:nTrials
    allData = [allData; data{jj,2}]
end

nData = length(allData);
A_ATI = trans(r+d)*A


for kk = 1:nData
    tensions(kk,:) = ( lsqnonneg(A_ATI,allData(kk,:)') )';
end
tensions
adhesiveLimit = max(tensions(:,1),tensions(:,2))
limitFit = ones(nData,1)\adhesiveLimit


figure; set(gca,'fontsize',16); hold on 
plot(1:nData,adhesiveLimit,'*')
plot(1:nData,limitFit*ones(1,nData))
%axis([0 nData+1 0 90])
axis([0 nData+1 0 30])
ylabel('Adhesive Failure [N]')
xlabel('Trial')
title('Adhesive Limit Fit')

%% Fit to two limits
adhesive1Fails = adhesiveLimit == tensions(:,1);
adhesive1Failure = adhesiveLimit(adhesive1Fails);
adhesive2Failure = adhesiveLimit(~adhesive1Fails);

n1 = length(adhesive1Failure);
n2 = length(adhesive2Failure);

limitFit1 = ones(n1,1)\adhesive1Failure
limitFit2 = ones(n2,1)\adhesive2Failure


%% Adhesive fit plot
figure; set(gca,'fontsize',16); hold on 
plot(1:n1,adhesive1Failure,'m*',(n1+1):(n1+n2),adhesive2Failure,'bo')
plot(1:n1,limitFit1*ones(1,n1),'m--',n1+(1:n2),limitFit2*ones(1,n2),'b--')
axis([0 nData+1 0 40])
ylabel('Adhesive Failure [N]')
xlabel('trial')
legend('Adhesive 1 failure','Adhesive 2 failure')

fig = gcf;
fig.PaperPositionMode = 'auto'
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

addpath('functionsCvx','functionsHelper','dataGenerated')
print(fig,'adhesionFailure','-dpdf')
%% Surface Plot At Wrist
load('3DmeshLimit')
limitWrist = (trans(r)*limit')';
ATI2Wrist = trans(-d);

figure; set(gca,'fontsize',16); hold on 


s = surf(f,m,FyWrist)

for nn = 1:nTrials
    numWrist = (ATI2Wrist*data{nn,2}')';
    scatter3(numWrist(:,1),numWrist(:,3),numWrist(:,2),60,'*')
end

%allDataWrist = (trans(r)*allData')';
%scatter3(allDataWrist(:,1),allDataWrist(:,3),allDataWrist(:,2),60,'*')
xlabel('F_x [N]')
zlabel('F_y [N]')
ylabel('T_z [Nm]')
title('Force/Torque at Wrist')
alpha(s,0.05) % transparency
%legend(['model' data{:,1}])
legend('model', 'pulloff tests')

%% Contour Plot At Wrist
figure; set(gca,'fontsize',16); hold on 
[X,Y] = meshgrid(f,m);
contour3(X,Y,FyWrist,10,'linewidth',4)
% for nn = 1:nTrials
%     numWrist = (ATI2Wrist*data{nn,2}')';
%     scatter3(numWrist(:,1),numWrist(:,3),numWrist(:,2),60,'*')
% end
xlabel('F_x [N]')
zlabel('F_y [N]')
ylabel('T_z [Nm]')
title('Force/Torque at Wrist')
%legend(['model' data{:,1}])
legend('model', 'pulloff tests')

%%
load('dataGenerated/3DscatterLimit_AsymmetricPaper_Sept8') % limit variable represents as accelerations about object's COM
limit(isinf(limit(:,3)),:) = [];% Get rid of erraneous vals
%limitWrist = (trans(r)*limit')';
limitWrist = limit;
figure; set(gca,'fontsize',16); hold on 
%s = surf(f,m,FyWrist)
allData = [];
for nn = 1:nTrials
    allData = [allData; data{nn,2}]; 
end
%allDataWrist = (ATI2Wrist*allData')';
allDataWrist = (ATI2Wrist*allData')';
scatter3(allDataWrist(:,1),allDataWrist(:,3),allDataWrist(:,2),60,'k*','LineWidth',1.5)
xlabel('F_x [N]')
zlabel('F_y [N]')
ylabel('T_z [Nm]')
title('Force/Torque at Wrist')
alpha(s,0.05) % transparency
%legend(['model' data{:,1}])

legend('Adhesive Failure')
axis tight
plotManualIsolines(limitWrist,limitWrist(:,2))


% fig = gcf;
% fig.PaperPositionMode = 'auto'
% fig_pos = fig.PaperPosition;
% fig.PaperSize = [fig_pos(3) fig_pos(4)];
% 
% addpath('functionsCvx','functionsHelper','dataGenerated')
% print(fig,'pullofftests','-dpdf')
