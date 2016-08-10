%% Previous Model 

% load('alpha20')
% load('alpha50')
% load('alpha80')
% 
% figure
% figure
% set(gca,'fontsize',16)
% hold on
% 
% plot(alpha80(:,1),alpha80(:,2),'*y')
% plot(alpha50(:,1),alpha50(:,2),'.k')
% plot(alpha20(:,1),alpha20(:,2),'*')
% legend('\alpha = 20','\alpha = 50','\alpha = 80')
% 
% title('Net Achievable Accelerations on Object')
% xlabel('ax [m/s^2]')
% ylabel('Rotational Accel_z [m/s^2]')

%% Looking at how alpha varies largest Fx you can achieve... 
% Note this is still assuming Fy is positive

alphas = 0:80';
r = 9/2*0.0254; % Distance from object COM to object surface
maxFx = zeros(size(alphas));

constraints = [20 20 100000000000 100000000000]';
nAlpha = numel(alphas);
for ii = 1:nAlpha
    disp(['Trial ' num2str(ii) ' of ' num2str(nAlpha)]); 
    A20 = defineGeometry(alphas(ii),r);
    [ fx, tensions, components ] = cvxGripMaxFx( A20, constraints);
    maxFx(ii) = fx; 
end

figure; set(gca,'fontsize',16); hold on; 
plot(alphas,maxFx,'Linewidth',3')
title('Net Achievable Accelerations on Object')
ylabel('Max F_x [N]')
xlabel('\alpha [deg]')

%% Look at how the 2D curve changes
alpha = 50;
r = 9/2*0.0254; % Distance from object COM to object surface
maxAdhesion = 20; 
constraints = [maxAdhesion maxAdhesion 10e6 10e6]';

A20 = defineGeometry(20,r);
A50 = defineGeometry(50,r);
A65 = defineGeometry(65,r);
A80 = defineGeometry(80,r);

% [fx_max, tensions, components ] = cvxGripMaxFx( A, constraints);

[limit20, tensions20] = convexGripperMz2DSweep(A20,maxAdhesion);
[limit50, tensions50] = convexGripperMz2DSweep(A50,maxAdhesion);
[limit65, tensions50] = convexGripperMz2DSweep(A65,maxAdhesion);
[limit80, tensions80] = convexGripperMz2DSweep(A80,maxAdhesion);

save('varyingAlpha','limit20','limit50','limit65','limit20')

% Overlay plots 
figure; set(gca,'fontsize',16); hold on; 
%plot(limit80(:,1),limit80(:,3),'*')
plot(limit65(:,1),limit65(:,3),'*')
plot(limit50(:,1),limit50(:,3),'*')
plot(limit20(:,1),limit20(:,3),'*')

title('Varying \alpha (assuming F_y = 0)')
xlabel('F_x [N]')
ylabel('T_z [Nm]')
legend('\alpha = 65 [deg]', '\alpha = 50 [deg]', '\alpha = 20 [deg]')

%% Show plots when taken about wrist 

% alpha = 50;
% r = 9/2*0.0254; % Distance from object COM to object surface
% maxAdhesion = 20; 
% constraints = [maxAdhesion maxAdhesion 10e6 10e6]';

trans = @(r) [1 0 0; 0 1 0; r 0 1];

limitWrist20 = (trans(r)*limit20')';
limitWrist50 = (trans(r)*limit50')';
limitWrist65 = (trans(r)*limit65')';

figure ; hold on
plot(limitWrist65(:,1),limitWrist65(:,3),'*')
plot(limitWrist50(:,1),limitWrist50(:,3),'*')
plot(limitWrist20(:,1),limitWrist20(:,3),'*')


