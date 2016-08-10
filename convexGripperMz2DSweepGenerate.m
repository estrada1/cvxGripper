%% Making sense of the shape of the limit curve
close all;
clear;
clc; 

% Define Parameters
alpha = 11.35;      % [deg]
%alpha = 50;
r = 9/2*0.0254;     % [m]
h = .009;           % [m] offset from surface
maxAdhesion = 19;

trans = @(rd) [1 0 0; 0 1 0; rd 0 1];

% Set up problem
A = defineGeometry(alpha,r);

[limit tensions] = convexGripperMz2DSweep(A,maxAdhesion); 

%% Plot
limitWrist = (trans(r)*limit')'
figure; hold on; set(gca,'fontsize',16);

% Hand-copied 
adhesive1 = [19 19 7.6171 0;...
            19 19 0 7.6171;...
            0 19 3.8235  0;...
            0 19 0 3.8235];
adhesiveLimit1 = (trans(r)*A*adhesive1')';

adhesive2 = [19 19 7.6171 0;...
            19 19 0 7.6171;...
            19 0 3.8235  0;...
            19 0 0 3.8235];
adhesiveLimit2 = (trans(r)*A*adhesive2')';
              
plot(limitWrist(:,1),limitWrist(:,3),'*')

% Hand-picked 
% specialInd = [1, 5, 55, 148,198,202];
tensions(specialInd,:)

specialInd = [1 5 55]; 
plot(adhesiveLimit2(:,1),adhesiveLimit2(:,3),'*','MarkerSize',15,'LineWidth',3)
plot(adhesiveLimit1(:,1),adhesiveLimit1(:,3),'o','MarkerSize',15,'LineWidth',3)
xlabel('F_x [N]')
ylabel('T_z [Nm]')
title('Projected Envelope Shape (F_y = 0)')
legend('Limit','Adhesive1 Fails','Adhesion2 Fails')

%%
figure
scatter3(limitWrist(:,1),limitWrist(:,3),1:length(limitWrist),'*')

%%
figure
plot(limit(:,1),limit(:,3),'*')
xlabel('F_x [N]')
ylabel('T_z [Nm]')
%% Visualize FBD 
figure; hold on; set(gca,'fontsize',16);
theta = 0:.1:pi; 
surf = [r*sin(theta)' r*cos(theta)']; % surface of circular object
contact = [r*sind(alpha) r*cosd(alpha); r*sind(-alpha) r*cosd(-alpha)];
    % [x1 y1; x2 y2]

scale = 1/600;
for ii = 1:length(limit)
    
    subplot(1,2,2)
    hold on; 
    
    t1 = scale*tensions(ii,1)*A(1:2,1); 
    t2 = scale*tensions(ii,2)*A(1:2,2);
    c1 = scale*tensions(ii,3)*A(1:2,3);
    c2 = scale*tensions(ii,4)*A(1:2,4);

    plot(surf(:,2),surf(:,1),'LineWidth',2)
    plot(contact(:,1),contact(:,2),'*','MarkerSize',12)
    quiver(contact(1,1), contact(1,2), t2(1), t2(2),'LineWidth',3)
    quiver(contact(2,1), contact(2,2), t1(1), t1(2),'LineWidth',3)
    quiver(contact(1,1), contact(1,2), c2(1), c2(2),'LineWidth',3)
    quiver(contact(2,1), contact(2,2), c1(1), c1(2),'LineWidth',3)


    win = 0.08; 
    axis([-win win -win+r win+r])

    pause(.251)
    clf
end


% %%
% % Plot sutff
% figure
% set(gca,'fontsize',16)
% hold on
% 
% xL = xlim;
% yL = ylim;
% line([0 0], yL);  %x-axis
% line(xL, [0 0]);  %y-axis
% 
% plot([Fx';Fx'],[Mz_max';Mz_min'],'*')
% xlabel('a_x [N]')
% ylabel('alpha_z [rad/sec^2]')
% title('Achievable Accelerations about object COM')
% 
% xL = xlim;
% yL = ylim;
% line([0 0], yL,'color','k','LineStyle','--');  % x-axis
% line(xL, [0 0],'color','k','LineStyle','--');  % y-axis
% 
% %% Translate this stuff
% FxMzWrist = trans*[Fx', zeros(length(Fx),1), Mz_max'; Fx', zeros(length(Fx),1), Mz_min']';
% FxMzWrist = FxMzWrist([1,3],:)';
% 
% wristx = 10;    % [N]
% wristz = .07;    % [Nm]
% wrist = [wristx wristz; wristx -wristz; -wristx -wristz; -wristx wristz];
% 
% figure
% set(gca,'fontsize',16)
% hold on
% 
% plot(FxMzWrist(:,1),FxMzWrist(:,2))
% plot(FxMzWrist(:,1),FxMzWrist(:,2),'*')
% xlabel('F_x [N]')
% ylabel('T_z [Nm]')
% title('Sustainable Force at Wrist')
% 
% %fill(wrist(:,1),wrist(:,2),'g')
% % xL = xlim;
% % yL = ylim;
% % line([0 0], yL,'color','k','LineStyle','--');  % x-axis
% % line(xL, [0 0],'color','k','LineStyle','--');  % y-axis
% 
% %%
% plot(-3,-1.4,'x','MarkerSize',10)
% point = [-3 0 -1.4]'
% x_point = A\inv(trans)*point
% x_nudge = -[x_point(1:4)]/norm(x_point(1:4))
% Fdir = point + trans*A*x_nudge
% 
% line([point(1); Fdir(1)],[point(3); Fdir(3)])
% 
% %% Translate a littler further
% d = r/20; 
% rd = r/cosd(alpha)
% trans2 = -[1 0 0 ; 0 1 0 ; (rd) 0 1];
% FxMzWrist2 = trans2*[Fx', zeros(length(Fx),1), Mz_max'; Fx', zeros(length(Fx),1), Mz_min']';
% FxMzWrist2 = FxMzWrist2([1,3],:)';
% 
% figure
% set(gca,'fontsize',16)
% hold on
% 
% plot(FxMzWrist2(:,1),FxMzWrist2(:,2))
% plot(FxMzWrist2(:,1),FxMzWrist2(:,2),'*')
% xlabel('F_x [N]')
% ylabel('T_z [Nm]')
% title('Sustainable Force at Wrist')
% % xL = xlim;
% % yL = ylim;
% % line([0 0], yL,'color','k','LineStyle','--');  % x-axis
% % line(xL, [0 0],'color','k','LineStyle','--');  % y-axis
% 
