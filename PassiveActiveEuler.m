%% PassiveActiveEuler.m
% Author: Matt Estrada
% Date: Sept 12, 2016
% Simulation of an active gripper vs. a Passive Gripper. 
% Uses Boyd's cvx toolbox for Matlab 
close all; clear all; clc; 

% Add boxes 
addpath('functionsCvx','functionsHelper','dataGenerated')
trans = @(rd) [1 0 0; 0 1 0; rd 0 1];

% Define Geometry 
alphad = 11.35;         % [deg]
r = 9/2*0.0254;         % [m]
Acm = defineGeometry(alphad,r); 
Awrist = trans(r)*Acm; 
A = Awrist; 

maxAdhesion1 =  24;
maxAdhesion2 = 19.28;
constraints = [maxAdhesion1; maxAdhesion2; 100; 100];

ITarzz                          =  0.0128;                  % kg*m^2              Constant
mTar                            =  1.5526;                 % kg                  Constant
r                               =  0.1143;                 % m                   Constant

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equations of Motion from Motion Genesis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

% F_applied = fx*nx> + fy*ny> 
Mn = @(theta) [1/mTar 0 0; ...
    0 1/mTar 0; ...
    -r*cos(theta)/ITarzz -r*sin(theta)/ITarzz 1/ITarzz];

% F_applied = fx*Tarx> + fy*Tary>
M = @(theta) [cos(theta)/mTar -sin(theta)/mTar 0; ...
    sin(theta)/mTar cos(theta)/mTar 0; ...
    -r/ITarzz 0 1/ITarzz];


A = [zeros(3) eye(3); zeros(3) zeros(3)];
%B = [zeros(3);M]

u = [0 0 0]';
%q0 = [0 0 0 0.5 -2 2*pi]';
q0 = [0 0 0 .3 -1 2*pi]';

n = 1000;
Q = zeros(6,n); 
q = q0; 

theta = q(3);
xp = q(4); 
yp = q(5);
thetap = q(6); 
minP = 0; 
    
xpTarB = sin(theta)*yp + cos(theta)*xp - r*thetap;
ypTarB = cos(theta)*yp - sin(theta)*xp;
KineticEnergy = 0.5*ITarzz*thetap^2 + 0.5*mTar*(xp^2+yp^2);
qpTarB = [xpTarB; ypTarB; q(6)];
dt = .001;


t = (0:n-1)*dt; 

for ii = 1:n
    KineticEnergy
    Q(:,ii) = q;
    QTarB(:,ii) = qpTarB;
    U(:,ii) = u; 
    K(:,ii) = KineticEnergy;    
    Pcalc(:,ii) = minP;
 
    
    if KineticEnergy < 0.05
        Q = Q(:,1:length(K));
        t = t(1:length(K)); 
        break
    end
    
    % Give easy names
    theta = q(3);
    xp = q(4); 
    yp = q(5);
    thetap = q(6); 
    
    % Calc B Matrix
    thisM = M(theta);
    B = [zeros(3); thisM];
    
    % Point of Contact 
    xpTarB = sin(theta)*yp + cos(theta)*xp - r*thetap;
    ypTarB = cos(theta)*yp - sin(theta)*xp;
    KineticEnergy = 0.5*ITarzz*thetap^2 + 0.5*mTar*(xp^2+yp^2);
    qpTarB = [xpTarB; ypTarB; q(6)];


    % Calculate Control law 
    % Minimize Power
%     [ minP, Fnet, tensions, components ] = cvxGripMinP( Awrist, constraints, qpTarB);
%     u = Fnet;

    % Rotation matrix from frame N to frame R 
    R_TN = [cos(theta), sin(theta), 0;  -sin(theta), cos(theta), 0;  0, 0, 1];
    % Oppose Velocity
    %[ beta, unit_vect, components ] = cvxGripBeta( Awrist, -qpTarB, constraints);
    [ beta, unit_vect, components ] = cvxGripBeta( Acm, -R_TN*q(4:6), constraints);
    u = trans(r)*components; 
    
    % Euler Method 
    dq = A*q + B*u;
    q = q + dq*dt;
end

%% Position, Velocity, Acceleration
figure
Q;
dQ = diff(Q,1,2);
dK = diff(K)/dt; 

% State
subplot(3,1,1)
plot(t,Q(1,:),t, Q(2,:),t,Q(3,:))
legend('x','y','theta')
title('Position')
subplot(3,1,2)
plot(t,Q(4,:),t, Q(5,:), t, Q(6,:))
legend('vx','vy','vtheta')
title('Velocity')
subplot(3,1,3)
plot(t(1:end-1),dQ(4,:),t(1:end-1),dQ(5,:),t(1:end-1),dQ(6,:))
legend('ax','ay','\alpha')
title('Acceleration')

% Kinetic Energy
figure
subplot(2,1,1)
plot(t,K)
title('Kinetic Energy')
subplot(2,1,2)
plot(t(1:end-1),dK)
title('Power')

% Comparing theoretical and simulated power 
figure
subplot(2,1,1)
plot(t,Pcalc)
title('Calculated Power')
subplot(2,1,2)
plot(t(1:end-1),dK)
title('Power')

%% Forces Applied
figure; 
plot(t,U(1,:),t, U(2,:), t, U(3,:))
legend('Fx','Fy','Mz')

figure
plot(t,QTarB(1,:),t, QTarB(2,:), t, QTarB(3,:))
title('Velocity of point')
legend('vxTar','vyTar','v \theta Tar')
%% Force Space
figure

plot3(U(1,:), U(2,:), U(3,:)); hold on; 
plot3(U(1,:), U(2,:), U(3,:),'.')


