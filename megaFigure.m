addpath('functionsCvx','functionsHelper','dataGenerated','megaFigure')

figure

%% Passive
    load('PASSIVE PaperRevisions_PassivePlot 23-Feb-2017 11-09-15')
    subplot(4,3,1); set(gca,'fontsize',16); hold on;
    [hAx,hLine1,hLine2] = plotyy(t,U(1:2,:),t,U(3,:));     
        % plotyy not recommended anymore, new function, 'yyaxis left/right' with
        % Matlab 2016B (I'm on 2014B at the moment)
        
    title('Passive')
    set(gca,'fontsize',14)
    legend('F_x','F_y','M_z')
    xlabel('time [sec]')
    ylabel(hAx(1),'Force [N]')
    ylabel(hAx(2),'Moment [Nm]','fontsize',16)

    subplot(4,3,4)
    set(gca,'fontsize',14); hold on;
    plot(t,K)
    xlabel('time [sec]')
    ylabel('K [J]')

    subplot(4,3,[7 10])
    set(gca,'fontsize',14); hold on;
    plotManualIsolines(limitWrist,limitWrist(:,2),'flipped')
    %plotManualIsolines(limitWrist,limitWrist(:,2))
    axis tight
    plot3(U(1,:), U(3,:), U(2,:),'LineWidth',3); hold on; 
    plot3(U(1,:), U(3,:), U(2,:),'ko','MarkerSize',10)
    grid on; 
    
%% Active

    load('ACTIVE PaperRevisions_ActivePlot 23-Feb-2017 09-31-52')
    subplot(4,3,[2]); set(gca,'fontsize',14); hold on;
    [hAx,hLine1,hLine2] = plotyy(t,U(1:2,:),t,U(3,:));     
        % plotyy not recommended anymore, new function, 'yyaxis left/right' with
        % Matlab 2016B (I'm on 2014B at the moment)
        
    legend('F_x','F_y','M_z')
    title('Active')
    xlabel('time [sec]')
    ylabel(hAx(1),'Force [N]')
    ylabel(hAx(2),'Moment [Nm]','fontsize',16)

    subplot(4,3,5)
    set(gca,'fontsize',14); hold on;
    plot(t,K)
    xlabel('time [sec]')
    ylabel('K [J]')

    subplot(4,3,[8 11])
    set(gca,'fontsize',14); hold on;
    plotManualIsolines(limitWrist,limitWrist(:,2),'flipped')
    %plotManualIsolines(limitWrist,limitWrist(:,2))
    axis tight
    plot3(U(1,:), U(3,:), U(2,:),'LineWidth',3); hold on; 
    plot3(U(1,:), U(3,:), U(2,:),'ko','MarkerSize',10)
    grid on; 
    
    %% Comparison 
subplot(4,3,[3 6 9 12])
    load('FirstDynamicSweep')
    Tar.r                               =  0.1143;                 % m                   Constant
    Tar.ITarzz                          =  0.0128;                 % kg*m^2              Constant
    Tar.mTar                            =  1.5526;                 % kg                  Constant
    massMatrix = [Tar.mTar 0 0; 0 Tar.mTar 0; 0 0 Tar.ITarzz]; 

    q0 = [0 -Tar.r 0 0 -0.2 2*pi]' ;
%     outcome = results(:,3)==1; 
%     indSuccess = find(results(:,3)==1);
%     indFail = find(results(:,3)==0);

    Knominal = 1/2*q0(4:6)'*massMatrix*q0(4:6); 

    %figure; set(gca,'fontsize',20); hold on;
    plot(results_PASSIVE(:,1).^2*Knominal,results_PASSIVE(:,3),'*');hold on;
    plot(results_ACTIVE(:,1).^2*Knominal,results_ACTIVE(:,3),'*');
    %scatter(results(indFail,1),results(indFail,2),'rx'); hold on;
    ylabel('t_{settle}');
    xlabel('K [J]'); 
    legend('Passive','Active')