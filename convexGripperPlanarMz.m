%% 2D Curved Surface Gripper
% Matt Estrada
% Going off Elliot's drawing in his curved gripper paper
% Doing a 2D slice to calculate result between moment/tangential force
% Super similar to convexGripperPlanar_LimitSurface_Cartesian, just
% ignoring the Fy direction
% June 29 2016


alpha = 11.35;      % [deg]
r = 9/2*0.0254;     % [m]
h = .009;           % [m] offset from surface
trans = [1 0 0; 0 1 0; r 0 1];

% Set up problem
A = defineGeometry(alpha,r);

maxAdhesion = 19; 
Fx = -maxAdhesion:1:maxAdhesion;
fy = 0;
constraints = [maxAdhesion; maxAdhesion; 1000000; 1000000];

objective = 'max';
Mz_max = zeros(size(Fx));

for nn = 1:numel(Fx)
    
    fx = Fx(nn);
    [Mz, vect] = cvxGripMz(alpha, r, fx, fy, constraints, objective);
    Mz_max(nn) = Mz;

    disp([ 'Trial ' num2str(nn) ' of ' num2str(numel(Fx))])

end

objective = 'min';
Mz_min = zeros(size(Fx));
for nn = 1:numel(Fx)
    
    fx = Fx(nn);
    [Mz, vect] = cvxGripMz(alpha, r, fx, fy, constraints, objective);
    Mz_min(nn) = Mz;

    disp([ 'Trial ' num2str(nn) ' of ' num2str(numel(Fx))])

end

%% Plot sutff
figure
set(gca,'fontsize',16)
hold on

xL = xlim;
yL = ylim;
line([0 0], yL);  %x-axis
line(xL, [0 0]);  %y-axis

plot([Fx';Fx'],[Mz_max';Mz_min'],'*')
xlabel('a_x [N]')
ylabel('alpha_z [rad/sec^2]')
title('Achievable Accelerations about object COM')

xL = xlim;
yL = ylim;
line([0 0], yL,'color','k','LineStyle','--');  % x-axis
line(xL, [0 0],'color','k','LineStyle','--');  % y-axis

%% Translate this stuff
FxMzWrist = trans*[Fx', zeros(length(Fx),1), Mz_max'; Fx', zeros(length(Fx),1), Mz_min']';
FxMzWrist = FxMzWrist([1,3],:)';

wristx = 10;    % [N]
wristz = .07;    % [Nm]
wrist = [wristx wristz; wristx -wristz; -wristx -wristz; -wristx wristz];

figure
set(gca,'fontsize',16)
hold on

plot(FxMzWrist(:,1),FxMzWrist(:,2))
plot(FxMzWrist(:,1),FxMzWrist(:,2),'*')
xlabel('F_x [N]')
ylabel('T_z [Nm]')
title('Sustainable Force at Wrist')

%fill(wrist(:,1),wrist(:,2),'g')
% xL = xlim;
% yL = ylim;
% line([0 0], yL,'color','k','LineStyle','--');  % x-axis
% line(xL, [0 0],'color','k','LineStyle','--');  % y-axis

%%
plot(-3,-1.4,'x','MarkerSize',10)
point = [-3 0 -1.4]'
x_point = A\inv(trans)*point
x_nudge = -[x_point(1:4)]/norm(x_point(1:4))
Fdir = point + trans*A*x_nudge

line([point(1); Fdir(1)],[point(3); Fdir(3)])

%% Translate a littler further
d = r/20; 
rd = r/cosd(alpha)
trans2 = -[1 0 0 ; 0 1 0 ; (rd) 0 1];
FxMzWrist2 = trans2*[Fx', zeros(length(Fx),1), Mz_max'; Fx', zeros(length(Fx),1), Mz_min']';
FxMzWrist2 = FxMzWrist2([1,3],:)';

figure
set(gca,'fontsize',16)
hold on

plot(FxMzWrist2(:,1),FxMzWrist2(:,2))
plot(FxMzWrist2(:,1),FxMzWrist2(:,2),'*')
xlabel('F_x [N]')
ylabel('T_z [Nm]')
title('Sustainable Force at Wrist')
% xL = xlim;
% yL = ylim;
% line([0 0], yL,'color','k','LineStyle','--');  % x-axis
% line(xL, [0 0],'color','k','LineStyle','--');  % y-axis



