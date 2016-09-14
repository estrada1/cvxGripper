%% PassiveActiveEuler.m
% Author: Matt Estrada, Hao Jiang
% Date: Sept 12, 2016
% Simulation of an active gripper vs. a Passive Gripper. 
% Uses Boyd's cvx toolbox for Matlab 
% Matt coded up active control, Hao coded up passive 
close all; clear all; clc; 



%% Set up Constants
% Rigid Body parameters 
    ITarzz                          =  0.0128;                  % kg*m^2              Constant
    mTar                            =  1.5526;                 % kg                  Constant
    r                               =  0.1143;                 % m                   Constant
    trans = @(rd) [1 0 0; 0 1 0; rd 0 1];
    addpath('functionsCvx','functionsHelper','dataGenerated')

    
%% Control Inputs here     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up Simulation and Initial Conditions here 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
PASSIVE_OR_ACTIVE = 'PASSIVE';
%PASSIVE_OR_ACTIVE = 'ACTIVE';
%q0 = [0 -r 0 .25 -0.25 .3]' % Set initial velocity with q0
%q0 = [0 -r 0 -0.25 -0.25 2*pi]' %< --- Good comparison , sort of quick-stop
q0 = [0 -r 0 0 -0.2 2*pi]' %< --- Good comparison 



%q0 = [0 -r 0 .3 -1 4*pi]'; <--- Good settings for aggressive active control
trialName = 'playing'; 
limitsurfaceFile = '3DscatterLimit_AsymmetricPaper_Sept8';

showplots = 1; 
    
%% Setting up More Constants 

% Limit Surface to Plot
load('3DscatterLimit_AsymmetricPaper_Sept8')
limit(isinf(limit(:,3)),:) = [];    % Get rid of erraneous vals
limit(isnan(limit(:,3)),:) = [];    % Get rid of erraneous vals
limitWrist = (trans(r)*limit')';
limitWrist=limit;


if strcmp(PASSIVE_OR_ACTIVE,'PASSIVE')

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    % Passive wrist stiffness
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    kx = 250;   % 250N/m
    ky = 250;   % 250N/m
    %kz = 0.125; % 0.125Nm
    kz = 0.125;
    % fake numbers for damping
    bx = 2;
    by = 2;
    bz = 0.002;
    
    elseif strcmp(PASSIVE_OR_ACTIVE,'ACTIVE')
        % Useful paths for convex optimization code
            trans = @(rd) [1 0 0; 0 1 0; rd 0 1];

        % Define Geometry of the gripper
            %These numbers are mainly useful for convex optimization 
            alphad = 11.35;         % [deg]
            r = 9/2*0.0254;         % [m]
            Acm = defineGeometry(alphad,r); 
            Awrist = trans(r)*Acm; 
            A = Awrist; 
            maxAdhesion1 =  24;
            maxAdhesion2 = 19.28;
            constraints = [maxAdhesion1; maxAdhesion2; 100; 100];
            
    else
        disp('SOMETHING WENT WRONG')
end

%% Define EOM
% Here I use a matrix form: 
% [ax ay alpha]' = M * [fx fy mz]'
% fx fy mz are force and moment applied as the gripper contact point
% acceleration is acceleration of the center of matt 
% Two different forms for matrix M, depending on which frame you express fx fy

if strcmp(PASSIVE_OR_ACTIVE,'PASSIVE')
    
    % F_applied = fx*nx> + fy*ny> 
    M = @(theta) [1/mTar 0 0; ...
        0 1/mTar 0; ...
        -r*cos(theta)/ITarzz -r*sin(theta)/ITarzz 1/ITarzz];
    
    elseif strcmp(PASSIVE_OR_ACTIVE,'ACTIVE')
        
    % F_applied = fx*Tarx> + fy*Tary>
    M = @(theta) [cos(theta)/mTar -sin(theta)/mTar 0; ...
        sin(theta)/mTar cos(theta)/mTar 0; ...
        -r/ITarzz 0 1/ITarzz];
end

% Equations of Motion from Motion Genesis
% EOM
%F_applied = fx*nx> + fy*ny>        force applied in fixed frame 
% xpp = fx/mTar;
% ypp = fy/mTar;
% thetapp = (mz-r*fx*cos(theta)-r*fy*sin(theta))/ITarzz;

% EOM 
% F_applied = fx*Tarx> + fy*Tary>   force in gripper frame
% xpp = (fx*cos(theta)-fy*sin(theta))/mTar;
% ypp = (fx*sin(theta)+fy*cos(theta))/mTar;
% thetapp = (mz-r*fx)/ITarzz;

%% Set up Matrix for EOM and integration
% Here I use the form: 
% d/dt(q) = A*q + B*u
% q is our state matrix, u is the external forces/control input
%   q = [x y theta x' y' theta']'
%       These state variable are for Tar_cm, rigid body Tar's center of mass 
%   u = [fx fy mz]' 
%       Applied at TarB, a point on Tar's surface where the gripper forces act

A = [zeros(3) eye(3); zeros(3) zeros(3)];



%q0 = [0 -r 0 .3 -1 4*pi]';  % <--- Remember last three quantities are velocities
                            % setting y0 = -r makes yTarB_0 = 0; 
                          
u0 = [0 0 0]';

n = 2000;   % Number of time steps to step through 
dt = .001;  % Step size

% Initial conditions
q = q0; u = u0; 

% Give intuitive names to some quantities 
% Paul uses notation xp = x' = vx 
x = q(1); y = q(2); theta = q(3); xp = q(4); yp = q(5); thetap = q(6); 
minP = 0; 
    
% Useful to track the velocity of point TarB in reference frame Tar , dubbed qpTarB
xpTarB_RefT = sin(theta)*yp + cos(theta)*xp - r*thetap; % In reference frame Tar
ypTarB_RefT = cos(theta)*yp - sin(theta)*xp;
KineticEnergy = 0.5*ITarzz*thetap^2 + 0.5*mTar*(xp^2+yp^2);
qpTarB = [xpTarB_RefT; ypTarB_RefT; q(6)];


% Position and Velocity of TarB in reference frame N 
% The gripper is attached to rigid body Tar at TarB 
xTarB_N = x - r*sin(theta);
yTarB_N = y + r*cos(theta);
xpTarB_N = xp - r*cos(theta)*thetap;
ypTarB_N = yp - r*sin(theta)*thetap;

% Time array 
t = (0:n-1)*dt; 

% Loop for simulation 
for ii = 1:n
    str = sprintf('t: %d \t K: %d',ii*dt,KineticEnergy);
    disp(str)

    %disp(['t: ' num2str(ii*dt) ',' '\t' 'K: ' num2str(KineticEnergy)])
     % Print out K so that we can tell the object is slowing down
    
    % Store useful quantities in a big matrix
    Q(:,ii) = q;
    QTarB(:,ii) = qpTarB;
    U(:,ii) = u; 
    K(:,ii) = KineticEnergy;    
    Pcalc(:,ii) = minP;
    
    if strcmp(PASSIVE_OR_ACTIVE,'ACTIVE')
        %Break loop if object has slowed down 
        %May want to remove this for passive gripper, since energy is still
        %stored in springs 
        if KineticEnergy < 0.02
            Q = Q(:,1:length(K));
            t = t(1:length(K)); 
            break
        end
    end
    
    % Give easy names
    % Give easy names
    x = q(1); y = q(2); theta = q(3); xp = q(4); yp = q(5); thetap = q(6); 
    
    % Calc B Matrix (changes with orientation 
    thisM = M(theta); % <-- code above should correspond to whether M is for active or passive forces
    B = [zeros(3); thisM];
    
    % Point of Contact velocity in Tar ref frame
    xpTarB_RefT = sin(theta)*yp + cos(theta)*xp - r*thetap;
    ypTarB_RefT = cos(theta)*yp - sin(theta)*xp;
    KineticEnergy = 0.5*ITarzz*thetap^2 + 0.5*mTar*(xp^2+yp^2);
    qpTarB = [xpTarB_RefT; ypTarB_RefT; q(6)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update x and y
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xTarB_N = x - r*sin(theta);
    yTarB_N = y + r*cos(theta);
    xpTarB_N = xp - r*cos(theta)*thetap;
    ypTarB_N = yp - r*sin(theta)*thetap;

    % Calculate External Force
    if strcmp(PASSIVE_OR_ACTIVE,'ACTIVE')

       % Calculate Control law 
        
%         % Minimize Power 
%             %this was doing some whacky things
%             [ minP, Fnet, tensions, components ] = cvxGripMinP( Awrist, constraints, qpTarB);
%             u = Fnet;
         
        % Oppose Momentum
        R_TN = [cos(theta), sin(theta), 0;  -sin(theta), cos(theta), 0;  0, 0, 1]; % Rotation matrix from frame N to frame R 
        massMatrix = [mTar 0 0; 0 mTar 0; 0 0 ITarzz]; 
        [ beta, unit_vect, components ] = cvxGripBeta( Acm, -massMatrix*R_TN*q(4:6), constraints);
        u = trans(r)*components; 

%             % Playing qpTarB
%            [ beta, unit_vect, components ] = cvxGripBeta( Awrist, -qpTarB, constraints);
%             u = components; 
        
    elseif strcmp(PASSIVE_OR_ACTIVE,'PASSIVE')
        
         % CALCULATE FORCES FOR PASSIVE GRIPPER HERE 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Calculated u
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        fx = -xTarB_N * kx + -xpTarB_N * bx;
        fy = -yTarB_N * ky + -ypTarB_N * by;
        mz = -theta * kz -thetap * bz;
        u = [fx fy mz]';
    end
    
    % Euler Method Integration actually happens here 
    dq = A*q + B*u;
    q = q + dq*dt;
end

%% Save Simulation Results 
results = struct('Q',q,'U',U,'t',t,'q0',q0,'limitWrist',limitWrist);
timenow = char(datetime('now','Format','dd-MMM-yyyy HH-mm-ss'));
save(['dataGenerated/' PASSIVE_OR_ACTIVE ' ' trialName ' ' timenow ])

%% Plotting 

if(showplots)
    % Position, Velocity, Acceleration
    figure; set(gca,'fontsize',16); hold on;
    Q;
    dQ = diff(Q,1,2);
    dK = diff(K)/dt; 

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
    figure
    subplot(2,1,1)
    plot(t,K)
    title('Kinetic Energy')
    subplot(2,1,2)
    plot(t(1:end-1),dK)
    title('Power')



    %%%%%%%% Forces Applied
    figure; set(gca,'fontsize',16); hold on;
    [hAx,hLine1,hLine2] = plotyy(t,U(1:2,:),t,U(3,:))     
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
    figure
    figure; set(gca,'fontsize',20); hold on;
    plotManualIsolines(limitWrist,limitWrist(:,2),'flipped')
    axis tight

    plot3(U(1,:), U(3,:), U(2,:),'LineWidth',3); hold on; 
    plot3(U(1,:), U(3,:), U(2,:),'ko','MarkerSize',10)

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