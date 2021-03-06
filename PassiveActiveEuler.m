function [Q,U,K,QTarB,t,limitWrist,tensions,success] = ...
    PassiveActiveEuler(Tar,q0,PASSIVE_OR_ACTIVE,tuning,trialName,Awrist,limits,limitsurfaceFile,printouts)

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


%% Set up Constants
% Rigid Body parameters 
ITarzz = Tar.ITarzz;
mTar = Tar.mTar;
r = Tar.r;
trans = @(rd) [1 0 0; 0 1 0; rd 0 1];

success = 2; % Weird case, this shouldn't happen

addpath('functionsCvx','functionsHelper','dataGenerated')
warning('off','MATLAB:nargchk:deprecated')
    
%% Control Inputs here     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up Simulation and Initial Conditions here 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%% Setting up More Constants 

% Limit Surface to Plot
load(limitsurfaceFile)
limit(isinf(limit(:,3)),:) = [];    % Get rid of erraneous vals
limit(isnan(limit(:,3)),:) = [];    % Get rid of erraneous vals
%limitWrist = (trans(r)*limit')';
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
        
        ITarzz_ContactPt = (ITarzz+mTar*r^2); 

        wnx = sqrt(kx/mTar) ;
        zetax = bx/(2*sqrt(mTar*kx));
        
        wnz = sqrt(kz/ITarzz_ContactPt);
        zetaz = bz/(2*sqrt(ITarzz_ContactPt*kz));
        
        % critically damped
        bx = 2*sqrt(mTar*kx);
        by = 2*sqrt(mTar*ky);
        %bz = 2*sqrt((ITarzz)*kz);
        bz = 2*sqrt((ITarzz+mTar*r^2)*kz);
        
        %% Trying with natural freq and damping ratio
        wn = tuning(1);
        zeta = tuning(2);
%         wn = 12; 
% %         wn = 5;
%         zeta = .4;
% %         zeta=1;  
% 
%         wnz = wn/6;
%         zetaz = zeta/3;    
        
        wnz = wn; 
        zetaz = zeta; 
        
        
        kx = wn^2*mTar;
        ky = wn^2*mTar; 
        kz = wnz^2*ITarzz_ContactPt;
        
        bx = 2*zeta*sqrt(mTar*kx); 
        by = 2*zeta*sqrt(mTar*ky); 
        bz = 2*zetaz*sqrt(ITarzz_ContactPt*kz); 
        
        
    
    case 'ACTIVE'
        disp('Active Simulation')
        % Useful paths for convex optimization code
        trans = @(rd) [1 0 0; 0 1 0; rd 0 1];

        % Define Geometry of the gripper
        %These numbers are mainly useful for convex optimization 
%         alphad = 11.35;         % [deg]
%         r = 9/2*0.0254;         % [m]
%         Acm = defineGeometry(alphad,r); 
%         Awrist = trans(r)*Acm; 
        A = Awrist; 
        maxAdhesion1 =  limits(1);
        maxAdhesion2 = limits(2);
        constraints = [maxAdhesion1; maxAdhesion2; 100; 100];
        
        % Linear Interpolation of Limit Surface
        %fit_polar = fitSurface_polar(limitWrist,'linearinterp') % Fitting surface w/ linear interpolation 
        %fit_polar = fitSurface_polar((trans(-r)*limitWrist')','linearinterp'); % Fitting surface w/ linear interpolation 

%         figure; 
%         plot3(limitWrist(:,1),limitWrist(:,2),limitWrist(:,3),'*'); hold on; 
%         plotManualIsolines(limitWrist,limitWrist(:,2));

            
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
        
%         M = @(theta) [1/mTar 0 0; ...
%             0 1/mTar 0; ...
%             -r*cos(theta)/ITarzz -r*sin(theta)/ITarzz 1/ITarzz];
        
        M = @(theta) [cos(theta)/mTar -sin(theta)/mTar 0; ...
            sin(theta)/mTar cos(theta)/mTar 0; ...
            -r/ITarzz 0 1/ITarzz];
    
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

n = 50000;   % Number of time steps to step through 
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
tensions = zeros(4,length(t)); 

% Loop for simulation 
for ii = 1:n
    
    if(printouts)
        str = sprintf('t: %d \t K: %d',ii*dt,KineticEnergy);
        disp(str)
    end
    
    % Store useful quantities in a big matrix (capital letters)
    Q(:,ii) = q;
    QTarB(:,ii) = qpTarB;
    U(:,ii) = u; 
    K(:,ii) = KineticEnergy;    
    Pcalc(:,ii) = minP;
    tensions(:,ii) = ( lsqnonneg(Awrist,u) )';

    % Check to see if adhesive limit is exceeded
    if strcmp(PASSIVE_OR_ACTIVE,'PASSIVE')
                    
        if (tensions(1,ii)>limits(1) || tensions(2,ii) > limits(2))
            Q = Q(:,1:length(K));
            t = t(1:length(K)); 
            tensions = tensions(:,1:length(K)); 
            success = 0; 
            break
        end
    end

    K_rest = K(1)*.02; 
    K_rest = .03; 
    if KineticEnergy < K_rest
        if strcmp(PASSIVE_OR_ACTIVE,'ACTIVE')
            Q = Q(:,1:length(K));
            t = t(1:length(K)); 
            tensions = tensions(:,1:length(K)); 
            success = 1; 
            break
        elseif(norm(u)<.01)
            
            endInd = find(K>K_rest,1,'last');
            Q = Q(:,1:endInd);
            K = K(:,1:endInd);
            t = t(1:endInd);
            QTarB = QTarB(:,1:endInd);
            U = U(:,1:endInd);
            Pcalc = Pcalc(:,1:endInd);
            tensions = tensions(:,1:endInd); 
            success = 1; 
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
    
    
    R_N2T = [cos(theta), sin(theta), 0;  -sin(theta), cos(theta), 0;  0, 0, 1]; % Rotation matrix from frame N to frame Tar 

    % Calculate Applied Force (from wrist)
    
    switch PASSIVE_OR_ACTIVE
        case 'ACTIVE'
            
           % Calculate Control law 

%              % Minimize Power 
%                  %this was doing some whacky things
%                  [ minP, Fnet, tensions, components ] = cvxGripMinP( Awrist, constraints, qpTarB);
%                  u = Fnet;

            massMatrix = [mTar 0 0; 0 mTar 0; 0 0 ITarzz]; 
            massMatrix_Wrist = [mTar 0 0; 0 mTar 0; 0 0 (ITarzz+mTar*r^2)];
            
            CONTROL_LAW = 'MOMENTUM_WRIST';
            switch CONTROL_LAW
                case 'MOMENTUM'
                % Oppose Momentum Control Law
                    massMatrix = [mTar 0 0; 0 mTar 0; 0 0 ITarzz]; 
                    [ beta, unit_vect, components ] = cvxGripBeta( Acm, -massMatrix*R_N2T*q(4:6), constraints);
                    u = trans(r)*components; 

                case 'MOMENTUM_WRIST'
                % Oppose Momentum Control Law
                    [ beta, unit_vect, components ] = cvxGripBeta( Awrist, -massMatrix_Wrist*qpTarB, constraints);
                    u = components; 

                case 'DAMPING'
                % Damping Control Law
                    [ beta, unit_vect, components ] = cvxGripBeta( Acm, -R_N2T*q(4:6), constraints);
                    u = trans(r)*components; 
                    
                case 'DAMPING_WRIST'
                % Damping Control Law
                    [ beta, unit_vect, components ] = cvxGripBeta( Awrist, -qpTarB, constraints);
                    u = components; 
                    
                case'RANGE_OF_MOTION'
                % Range of Motion 
                    RangeOfMotion = [1/0.04 0 0; 0 1/.04 0; 0 0 1/1.345];
                    [ beta, unit_vect, components ] = cvxGripBeta( Acm, -R_N2T*RangeOfMotion*q(4:6), constraints);
                    u = trans(r)*components; 

                case 'POWER'
                    [ minP, Fnet, tensions, components ] = cvxGripMinP( Acm, constraints, R_N2T*q(4:6));
%                  u = Fnet;
                    u = trans(r)*components; 

                case 'INTERPOLATION'
                    massMatrix = [mTar 0 0; 0 mTar 0; 0 0 ITarzz]; 
                    [mag, components] = solveSurface_polar(fit_polar,(-massMatrix*R_N2T*q(4:6))')
                    % [mag, components] = solveSurface_polar(fit_polar,(-[xpTarB_RefT xpTarB_RefT q(6)]')')
                    % components = mag*(-massMatrix*R_TN*q(4:6));
                    % components = trans(r)*comp; 
                    u = trans(r)*components; 

    
            end


        case 'PASSIVE'
        
             % CALCULATE FORCES FOR PASSIVE GRIPPER HERE 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Calculated u
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            fx = -xTarB_N * kx + -xpTarB_N * bx;
            fy = -yTarB_N * ky + -ypTarB_N * by;
            mz = -theta * kz -thetap * bz;
            u = R_N2T*[fx fy mz]';
            
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
