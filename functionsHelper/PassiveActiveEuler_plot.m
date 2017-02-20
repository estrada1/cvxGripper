function PassiveActiveEuler_plot(Q,U,K,QTarB,t,limitWrist)

    % Position, Velocity, Acceleration
    figure; set(gca,'fontsize',16); hold on;
    dt = t(2)-t(1); 
    dQ = diff(Q,1,2);
    dK = diff(K)/dt; 
    px =  QTarB(1,:).*U(1,:); % Power into system joint x
    py =  QTarB(2,:).*U(2,:); % joint y
    pz =  QTarB(3,:).*U(3,:); % joint z 

    %%%%%%%% States
    subplot(3,1,1); set(gca,'fontsize',16); hold on;
    plot(t,Q(1,:),t, Q(2,:),t,Q(3,:))
    legend('x','y','theta')
    title('Position')
    subplot(3,1,2); set(gca,'fontsize',16); hold on;
    plot(t,Q(4,:),t, Q(5,:), t, Q(6,:))
    legend('vx','vy','vtheta')
    title('Velocity')
    subplot(3,1,3); set(gca,'fontsize',16); hold on;
    plot(t(1:end-1),dQ(4,:),t(1:end-1),dQ(5,:),t(1:end-1),dQ(6,:))
    legend('ax','ay','\alpha')
    title('Acceleration')

    %%%%%%%% Kinetic Energy and Power 
    figure; 
    subplot(2,1,1); set(gca,'fontsize',16); hold on;
    plot(t,K)
    ylabel('Kinetic Energy [J]')
    subplot(2,1,2); set(gca,'fontsize',16); hold on;
    plot(t(1:end-1),dK,t,px,'--',t,py,'--',t,pz,'--')
    ylabel('Power [W]')
    xlabel('Time [sec]')
    legend('Total','JointX','JointY','JointZ')

    %%%%%%%% Forces Applied
    figure; set(gca,'fontsize',16); hold on;
    [hAx,hLine1,hLine2] = plotyy(t,U(1:2,:),t,U(3,:));     
        % plotyy not recommended anymore, new function, 'yyaxis left/right' with
        % Matlab 2016B (I'm on 2014B at the moment)
    legend('F_x','F_y','M_z')
    title('Force Applied at the gripper contact point')
    xlabel('time [sec]')
    ylabel(hAx(1),'Force [N]')
    ylabel(hAx(2),'Moment [Nm]','fontsize',16)

    figure; set(gca,'fontsize',16); hold on;
    plot(t,QTarB(1,:),t, QTarB(2,:), t, QTarB(3,:))
    title('Velocity of point')
    legend('vxTar','vyTar','v \theta Tar')
    xlabel('time [sec]')
    ylabel('velocity [m/s]')


    %%%%%%%% Force Space
    figure; set(gca,'fontsize',20); hold on;
    plotManualIsolines(limitWrist,limitWrist(:,2),'flipped')

    axis tight
    plot3(U(1,:), U(3,:), U(2,:),'LineWidth',3); hold on; 
    plot3(U(1,:), U(3,:), U(2,:),'ko','MarkerSize',10)
    % plot3(U(1,:), zeros(size(U(3,:))), U(2,:),'g.','MarkerSize',10) % Passive Project for 1 DOF actuation 

    %%%%%%%% MEGA FIGURE 
    figure;
    subplot(4,1,1); set(gca,'fontsize',16); hold on;
    [hAx,hLine1,hLine2] = plotyy(t,U(1:2,:),t,U(3,:));     
        % plotyy not recommended anymore, new function, 'yyaxis left/right' with
        % Matlab 2016B (I'm on 2014B at the moment)
        
    legend('F_x','F_y','M_z')
    title('Force Applied at the gripper contact point')
    xlabel('time [sec]')
    ylabel(hAx(1),'Force [N]')
    ylabel(hAx(2),'Moment [Nm]','fontsize',16)

    subplot(4,1,2)
    set(gca,'fontsize',14); hold on;
    plot(t,K)
    xlabel('time [sec]')
    legend('Kinetic Energy')

    subplot(4,1,3:4)
    set(gca,'fontsize',14); hold on;
    plotManualIsolines(limitWrist,limitWrist(:,2),'flipped')
    %plotManualIsolines(limitWrist,limitWrist(:,2))
    axis tight
    plot3(U(1,:), U(3,:), U(2,:),'LineWidth',3); hold on; 
    plot3(U(1,:), U(3,:), U(2,:),'ko','MarkerSize',10)


    % % Save Mega Figure
    % 
    % fig = gcf;
    % fig.PaperPositionMode = 'auto'
    % fig_pos = fig.PaperPosition;
    % fig.PaperSize = [fig_pos(3) fig_pos(4)];
    % 
    % addpath('functionsCvx','functionsHelper','dataGenerated')
    % print(fig,'ActiveGrasp','-dpdf')
end