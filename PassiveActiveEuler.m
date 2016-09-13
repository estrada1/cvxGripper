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

% theta                           =  0;                      % rad                 Initial Value
% x                               =  0;                   % m                   Initial Value
% y                               =  0;                      % m                   Initial Value
% thetap                          =  0;                      % rad/sec             Initial Value
% xp                              =  0.3;                    % m/s                 Initial Value
% yp                              =  -1.3;                    % m/s                 Initial Value



% EOM
% xpp = fx/mTar;
% ypp = fy/mTar;
% thetapp = (mz-r*fx*cos(theta)-r*fy*sin(theta))/ITarzz;

M = @(theta) [1/mTar 0 0; ...
    0 1/mTar 0; ...
    -r*cos(theta)/ITarzz -r*sin(theta)/ITarzz 1/ITarzz];

A = [zeros(3) eye(3); zeros(3) zeros(3)];
%B = [zeros(3);M]

u = [0 0 0]';
%q0 = [0 0 0 0.5 -2 2*pi]';
q0 = [0 0 0 0.5 -2 0]';

n = 1000;
Q = zeros(6,n); 
q = q0; 

    theta = q(3);
    xp = q(4); 
    yp = q(5);
    thetap = q(6); 
    
xpTarB = sin(theta)*yp + cos(theta)*xp - r*thetap;
ypTarB = cos(theta)*yp - sin(theta)*xp;
KineticEnergy = 0.5*ITarzz*thetap^2 + 0.5*mTar*(xp^2+yp^2);
qpTarB = [xpTarB; ypTarB; q(6)];
dt = .0005;

t = (0:n-1)*dt; 

for ii = 1:n
    KineticEnergy
    Q(:,ii) = q;
    QTarB(:,ii) = qpTarB;
    U(:,ii) = u; 
    K(:,ii) = KineticEnergy; 
    
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
    
    % Covex Optimization
    [ minP, Fnet, tensions, components ] = cvxGripMinP( Awrist, constraints, qpTarB);
    u = Fnet;
    
    % Euler Method 
    dq = A*q + B*u;
    q = q + dq*dt;
end
%%
Q;
subplot(3,1,1)
plot(t,Q(1,:),t, Q(2,:),t,Q(3,:))
legend('x','y','theta')
subplot(3,1,2)
plot(t,Q(4,:),t, Q(5,:), t, Q(6,:))
legend('vx','vy','vtheta')
subplot(3,1,3)
plot(t,K)
legend('KineticEnergy')
%%
figure; 
plot(t,U(1,:),t, U(2,:), t, U(3,:))
legend('Fx','Fy','Mz')
%%
figure
scatter(U(1,:), U(2,:), U(3,:),'*')

