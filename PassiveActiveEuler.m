%% PassiveActiveEuler.m
% Author: Matt Estrada, Hao Jiang
% Date: Sept 12, 2016
% Simulation of an active gripper vs. a Passive Gripper. 
% Uses Boyd's cvx toolbox for Matlab 
% Matt coded up active control, Hao coded up passive 

% Dynamics EOM are described in PassiveActiveGripper.txt, probably in my
% Motion Genesis application folder
% Simulating a single, translating, rotating, rigid body in 2D 
% NewtonianFrame  N                % Newtonian reference frame
% RigidBody       Tar              % Technical name of body
% Point           TarB( Tar )      % Point of applied gripper forces

close all; clear; clc; 

%% Set up Constants
% Rigid Body parameters 
    ITarzz                          =  0.0128;                 % kg*m^2              Constant
    mTar                            =  1.5526;                 % kg                  Constant
    r                               =  0.1143;                 % m                   Constant
    trans = @(rd) [1 0 0; 0 1 0; rd 0 1];
    addpath('functionsCvx','functionsHelper','dataGenerated')

    
%% Control Inputs here     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up Simulation and Initial Conditions here 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%PASSIVE_OR_ACTIVE = 'PASSIVE';
PASSIVE_OR_ACTIVE = 'ACTIVE';
%PASSIVE_OR_ACTIVE = 'ONE_DOF' 

%q0 = [0 -r 0 .25 -0.25 .3]' % Set initial velocity with q0
%q0 = [0 -r 0 -0.25 -0.25 2*pi]' %< --- Good comparison , sort of quick-stop
q0 = [0 -r 0 0 -0.2 2*pi]' %< --- Good comparison, used in paper submission
%q0 = [0 -r 0 0 -0.2 3*pi]' %< --- Good comparison, used in paper draft

%q0 = [0 -r 0 .3 -1 4*pi]'; <--- Good settings for aggressive active control
trialName = 'NeginAdapt_Oct10_MoreAngularMomentum'; 
limitsurfaceFile = '3DscatterLimit_AsymmetricPaper_Sept8';
showplots = 1; 
    
%% Setting up More Constants 

% Limit Surface to Plot
load('3DscatterLimit_AsymmetricPaper_Sept8')
limit(isinf(limit(:,3)),:) = [];    % Get rid of erraneous vals
limit(isnan(limit(:,3)),:) = [];    % Get rid of erraneous vals
limitWrist = (trans(r)*limit')';
limitWrist=limit;

%if strcmp(PASSIVE_OR_ACTIVE,'PASSIVE')
switch  PASSIVE_OR_ACTIVE
    case 'PASSIVE'
        
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
        % critically damped
        bx = 2*sqrt(mTar*kx);
        by = 2*sqrt(mTar*ky);
        bz = 2*sqrt(ITarzz*kz);
    
    case 'ACTIVE'
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
        
        % Linear Interpolation of Limit Surface
        fit_polar = fitSurface_polar(limit,'linearinterp'); % Fitting surface w/ linear interpolation 

            
    case 'ONE_DOF'
        % one Degree of freedom
        kx = 250;   % 250N/m
        ky = 250;   % 250N/m
        %kz = 0.125; % 0.125Nm
        kz = 0.125;
        % fake numbers for damping
        bx = 2;
        by = 2;
        bz = 0.002;
        
        alphad = 11.35;         % [deg]
        r = 9/2*0.0254;         % [m]
        Acm = defineGeometry(alphad,r); 
        Awrist = trans(r)*Acm; 
        A = Awrist; 
        maxAdhesion1 =  24;
        maxAdhesion2 = 19.28;
        constraints = [maxAdhesion1; maxAdhesion2; 100; 100];
end

%% Define EOM
% Here I use a matrix form: 
% [ax ay alpha]' = M * [fx fy mz]'
% fx fy mz are force and moment applied as the gripper contact point
% acceleration is acceleration of the center of mass
% Two different forms for matrix M, depending on which frame you express fx fy

switch PASSIVE_OR_ACTIVE
    case 'PASSIVE'
        % F_applied = fx*nx> + fy*ny> 
        M = @(theta) [1/mTar 0 0; ...
            0 1/mTar 0; ...
            -r*cos(theta)/ITarzz -r*sin(theta)/ITarzz 1/ITarzz];
    
    case 'ACTIVE'
        % F_applied = fx*Tarx> + fy*Tary>
        M = @(theta) [cos(theta)/mTar -sin(theta)/mTar 0; ...
            sin(theta)/mTar cos(theta)/mTar 0; ...
            -r/ITarzz 0 1/ITarzz];
        
    case 'ONE_DOF'
        % F_applied = fx*nx> + fy*ny> 
        M = @(theta) [1/mTar 0 0; ...
            0 1/mTar 0; ...
            -r*cos(theta)/ITarzz -r*sin(theta)/ITarzz 1/ITarzz];
        
end

% Equations of Motion from Motion Genesis
% EOM
% F_applied = fx*nx> + fy*ny>        force applied in fixed frame 
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
%   q = [x y theta x' y' theta']' in reference frame N 
%       These state variable are for Tar_cm, rigid body Tar's center of mass 
%   u = [fx fy mz]' 
%       Applied at TarB, a point on Tar's surface where the gripper forces act
%   Reference frame N 
%

A = [zeros(3) eye(3); zeros(3) zeros(3)];

%q0 = [0 -r 0 .3 -1 4*pi]';  % <--- Remember last three quantities are velocities
                            % setting y0 = -r makes yTarB_0 = 0; 
                          
u0 = [0 0 0]';

n = 1000;   % Number of time steps to step through 
dt = .0005;  % Step size

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
    
    % Store useful quantities in a big matrix (capital letters)
    Q(:,ii) = q;
    QTarB(:,ii) = qpTarB;
    U(:,ii) = u; 
    K(:,ii) = KineticEnergy;    
    Pcalc(:,ii) = minP;
    
    if strcmp(PASSIVE_OR_ACTIVE,'ACTIVE')
        %Break loop if object has slowed down 
        %May want to remove this for passive gripper, since energy is still
        %stored in springs 
        if KineticEnergy < 0.0001
            Q = Q(:,1:length(K));
            t = t(1:length(K)); 
            break
        end
    end
    
    % Give easy names
    x = q(1); y = q(2); theta = q(3); xp = q(4); yp = q(5); thetap = q(6); 
    
    % Calc B Matrix (changes with orientation 
    thisM = M(theta); % <-- M matrix should correspond to whether forces are expressed in N or Tar coordinates
    B = [zeros(3); thisM];
    
    % Point of Contact velocity in Tar ref frame
    xpTarB_RefT = sin(theta)*yp + cos(theta)*xp - r*thetap;
    ypTarB_RefT = cos(theta)*yp - sin(theta)*xp;
    KineticEnergy = 0.5*ITarzz*thetap^2 + 0.5*mTar*(xp^2+yp^2);
    qpTarB = [xpTarB_RefT; ypTarB_RefT; q(6)];
    xTarB_N = x - r*sin(theta);
    yTarB_N = y + r*cos(theta);
    xpTarB_N = xp - r*cos(theta)*thetap;
    ypTarB_N = yp - r*sin(theta)*thetap;

    % Calculate Applied Force (from wrist)
    
    switch PASSIVE_OR_ACTIVE
        case 'ACTIVE'
            
           % Calculate Control law 

%              % Minimize Power 
%                  %this was doing some whacky things
%                  [ minP, Fnet, tensions, components ] = cvxGripMinP( Awrist, constraints, qpTarB);
%                  u = Fnet;

            R_TN = [cos(theta), sin(theta), 0;  -sin(theta), cos(theta), 0;  0, 0, 1]; % Rotation matrix from frame N to frame Tar 
            
            CONTROL_LAW = 'INTERPOLATION';
            switch CONTROL_LAW
                case 'MOMENTUM'
                % Oppose Momentum Control Law
                    massMatrix = [mTar 0 0; 0 mTar 0; 0 0 ITarzz]; 
                    [ beta, unit_vect, components ] = cvxGripBeta( Acm, -massMatrix*R_TN*q(4:6), constraints);
                
                case 'DAMPING'
                % Damping Control Law
                    [ beta, unit_vect, components ] = cvxGripBeta( Acm, -R_TN*q(4:6), constraints);
               
                case'RANGE_OF_MOTION'
                % Range of Motion 
                    RangeOfMotion = [1/0.04 0 0; 0 1/.04 0; 0 0 1/1.345];
                    [ beta, unit_vect, components ] = cvxGripBeta( Acm, -R_TN*RangeOfMotion*q(4:6), constraints);
                
                case 'POWER'
                    [ minP, Fnet, tensions, components ] = cvxGripMinP( Acm, constraints, R_TN*q(4:6));
%                  u = Fnet;

                case 'INTERPOLATION'
                    massMatrix = [mTar 0 0; 0 mTar 0; 0 0 ITarzz]; 
                    mag = solveSurface_polar(fit_polar,(massMatrix*R_TN*q(4:6))')
                    components = mag*(-massMatrix*R_TN*q(4:6))

    
            end
            u = trans(r)*components; 


        case 'PASSIVE'
        
             % CALCULATE FORCES FOR PASSIVE GRIPPER HERE 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Calculated u
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            fx = -xTarB_N * kx + -xpTarB_N * bx;
            fy = -yTarB_N * ky + -ypTarB_N * by;
            mz = -theta * kz -thetap * bz;
            u = [fx fy mz]';
            
        case 'ONE_DOF'
            
            fx = -xTarB_N * kx + -xpTarB_N * bx;
            fy = -yTarB_N * ky + -ypTarB_N * by;
            if  thetap < 0          % We want moment to oppose motion at that joint
                objective = 'max';
            else
                objective = 'min';
            end
            [ mz, vect ] = cvxGripMz( Awrist, fx, fy, constraints, objective ); % Requires cvx 
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
    figure; set(gca,'fontsize',20); hold on;
    %plotManualIsolines(limitWrist,limitWrist(:,2),'flipped')
        plotManualIsolines(limitWrist,limitWrist(:,2))

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