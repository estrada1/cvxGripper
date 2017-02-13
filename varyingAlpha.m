%% Look at how the 2D curve changes
% Visualize how the limit surface changes as we change our alpha parameter
% (how much the gripper engulfs the object) 

generateData = 0; % toggle whether we want to generate the data too 

if generateData
    
    defineParameters; 

    A20 = defineGeometry(20,r);
    A50 = defineGeometry(50,r);
    A65 = defineGeometry(65,r);
    A80 = defineGeometry(80,r);

    % [fx_max, tensions, components ] = cvxGripMaxFx( A, constraints);
    fy = 0; 

    [limit20, tensions20] = limitSurfaceMz2D(A20,constraints, fy);
    [limit50, tensions50] = limitSurfaceMz2D(A50,constraints, fy);
    [limit65, tensions50] = limitSurfaceMz2D(A65,constraints, fy);
    [limit80, tensions80] = limitSurfaceMz2D(A80,constraints, fy);

    save('varyingAlpha_Sept8','limit20','limit50','limit65','limit20')
else
    
    load('varyingAlpha_Sept8')
    
end
%% Overlay plots 
figure; set(gca,'fontsize',20); hold on; 
%plot(limit80(:,1),limit80(:,3),'*')
plot(limit65(:,1),limit65(:,3),':','LineWidth',4)
plot(limit50(:,1),limit50(:,3),'--','LineWidth',4)
plot(limit20(:,1),limit20(:,3),'-','LineWidth',4)

%title('Varying \alpha (assuming F_y = 0)')
xlabel('F_x [N]')
ylabel('M_z [Nm]')
legend('\alpha = 65 [deg]', '\alpha = 50 [deg]', '\alpha = 20 [deg]')

fig = gcf;
fig.PaperPositionMode = 'auto'
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

addpath('functionsCvx','functionsHelper','dataGenerated')
print(fig,'varyingAlpha','-dpdf')



%% Looking at how alpha varies largest Fx you can achieve... 
% % Note this is still assuming Fy is positive
% 
% alphas = 0:80';
% r = 9/2*0.0254; % Distance from object COM to object surface
% maxFx = zeros(size(alphas));
% 
% constraints = [20 20 100000000000 100000000000]';
% nAlpha = numel(alphas);
% for ii = 1:nAlpha
%     disp(['Trial ' num2str(ii) ' of ' num2str(nAlpha)]); 
%     A20 = defineGeometry(alphas(ii),r);
%     [ fx, tensions, components ] = cvxGripMaxFx( A20, constraints);
%     maxFx(ii) = fx; 
% end
% 
% figure; set(gca,'fontsize',16); hold on; 
% plot(alphas,maxFx,'Linewidth',3')
% title('Net Achievable Accelerations on Object')
% ylabel('Max F_x [N]')
% xlabel('\alpha [deg]')