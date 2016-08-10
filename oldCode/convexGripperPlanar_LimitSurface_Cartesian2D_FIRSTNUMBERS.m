%% 2D Curved Surface Gripper
% Matt Estrada
% Going off Elliot's drawing in his curved gripper paper
% Doing a 2D slice to calculate result between moment/tangential force
% Super similar to convexGripperPlanar_LimitSurface_Cartesian, just
% ignoring the Fy direction
% June 29 2016


alpha = 15;     % [deg]
%alpha = 45;
r = 0.100;      % [meter]

trans = -[1 0 0 ; 0 1 0 ; r 0 1];


% Set up problem
t1 = [cosd(alpha); sind(alpha); - r];
t2 = [-cosd(alpha); sind(alpha); r];
c1 = [sind(alpha); -cosd(alpha); 0];
c2 = [-sind(alpha); -cosd(alpha); 0];
A =  [t1 t2 c1 c2];

%[Fx Mz] = meshgrid(-7:1:7,-0.6:0.1:0.6);
Fx = -20:.1:20;
Mz_max = zeros(size(Fx));

for nn = 1:numel(Fx)
    
    cvx_begin quiet

        variable x(4,1)
        variable mz(1,1)

        fx = Fx(nn);
        fy = 0;

        maximize( mz )

        subject to

%            x<=[10; 10; 10000; 10000]
            x<=[10; 20; 10000; 10000]

            x>=[0; 0; 0; 0]
            [fx fy mz]' ==  A * x

    cvx_end
    
    Mz_max(nn) = cvx_optval;

    disp([ 'Trial ' num2str(nn) ' of ' num2str(numel(Fx))])


end

Mz_min = zeros(size(Fx));

for nn = 1:numel(Fx)
    
    cvx_begin quiet

        variable x(4,1)
        variable mz(1,1)

        fx = Fx(nn);
        fy = 0;

        minimize( mz )

        subject to

            x<=[10; 20; 10000; 10000]
            x>=[0; 0; 0; 0]
            [fx fy mz]' ==  A * x

    cvx_end
    
    Mz_min(nn) = cvx_optval;

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

figure
set(gca,'fontsize',16)
hold on

plot(FxMzWrist(:,1),FxMzWrist(:,2))
plot(FxMzWrist(:,1),FxMzWrist(:,2),'*')
xlabel('F_x [N]')
ylabel('T_z [Nm]')
title('Sustainable Force at Wrist')
% xL = xlim;
% yL = ylim;
% line([0 0], yL,'color','k','LineStyle','--');  % x-axis
% line(xL, [0 0],'color','k','LineStyle','--');  % y-axis

% Translate a littler further
d = r/20; 
trans2 = -[1 0 0 ; 0 1 0 ; (r+d) 0 1];

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

