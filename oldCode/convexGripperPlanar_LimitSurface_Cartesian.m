%% 3D Curved Surface Gripper
% Going off Elliot's drawing in his curved gripper paper
% Doing a little different calculation for a meshgrid plot
% Matt Estrada
% June 24 2016

% limit = zeros(length(sweep_theta),1); 
% tension = zeros(length(sweep_theta),4); 
% limit3 = zeros(length(sweep_theta),3);

alpha = 15;     % [deg]
r = 0.100;      % [meter]

% Set up problem
t1 = [cosd(alpha); sind(alpha); - r];
t2 = [-cosd(alpha); sind(alpha); r];
c1 = [sind(alpha); -cosd(alpha); 0];
c2 = [-sind(alpha); -cosd(alpha); 0];
A =  [t1 t2 c1 c2];

%[Fx Mz] = meshgrid(-7:1:7,-0.6:0.1:0.6);
[Fx Mz] = meshgrid(-3:.1:3,-0.2:0.05:0.2);
Fy = zeros(size(Fx));


for nn = 1:numel(Fx)
    
    cvx_begin quiet

        variable x(4,1)
        variable fy(1,1)

        fx = Fx(nn);
        mz = Mz(nn);
        
        maximize( fy )

        subject to

            x<=[10; 10; 10000; 10000]
            x>=[0; 0; 0; 0]
            [fx fy mz]' ==  A * x

    cvx_end
    
    Fy(nn) = cvx_optval;

    disp([ 'Trial ' num2str(nn) ' of ' num2str(numel(Fx))])


end



%% Plot stuff

Fy_filt = Fy
Fy_filt(Fy_filt<=0) = 0
figure
set(gca,'fontsize',16)
hold on

surf(Fx,Fy_filt,Mz)
xlabel('Fx [N]')
ylabel('Fy [N]')
zlabel('Mz [Nm]')

