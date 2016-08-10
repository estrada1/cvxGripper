% Testing CVX
% Boyd's example code below
m = 20; n = 10; p = 4;
A = randn(m,n); b = randn(m,1);
C = randn(p,n); d = randn(p,1); e = rand;
cvx_begin
    variable x(n)
    minimize( norm( A * x - b, 2 ) )
    subject to
        C * x == d
        norm( x, Inf ) <= e
cvx_end

%% Simple Flat Gripper
sweep = 15:5:165'; 
%deg = 80
limit = zeros(length(sweep),1); 
tension = zeros(length(sweep),2); 

t1 = [-cosd(12) sind(12)];
t2 = [cosd(12) sind(12)];
A =  [t1;t2];
for n = 1:length(sweep)
    theta = sweep(n);
    d = [cosd(theta)  sind(theta)];
end

limit
tension

close all 

xy = [limit.*cosd(sweep)' limit.*sind(sweep)'];

figure
plot(xy(:,1),xy(:,2),'*')

figure
plot(sweep,limit)

%% 2D Curved Surface Gripper
% Going off Elliot's drawing in his curved gripper paper

sweep = 0:5:90; % DOUBLE CHECK IF THESE DIRECTIONS MAKE SENSE
d2 = [cosd(sweep')  zeros(length(sweep),1) sind(sweep') ]
limit = zeros(length(sweep),1); 
tension = zeros(length(sweep),4); 
limit3 = zeros(length(sweep),3);
alpha = 15;     % [deg]
theta = 90;     
r = 0.100;      % [meter]
t1 = [cosd(alpha); sind(alpha); - r];
t2 = [-cosd(alpha); sind(alpha); r];
c1 = [sind(alpha); -cosd(alpha); 0];
c2 = [-sind(alpha); -cosd(alpha); 0];

A =  [t1 t2 c1 c2];

%for n = 1:length(theta)
%    d = [sind(theta);  cosd(theta); 0 ];
for n = 1:length(sweep)
    
    th = sweep(n); 
    d = [cosd(th);  0; sind(th) ];
    
    cvx_begin
    
        variable x(4,1)
        variable fnet(1,1)
        
        maximize( fnet )
        
        subject to
        
            x<=[10; 10; 10000; 10000]
            x>=[0; 0; 0; 0]
            fnet*d ==  A * x
            
    cvx_end
    
   limit(n) = cvx_optval ;
   tension(n,:) = x;
   limit3(n,:) = A*x; 
   
end

closeForm = 2*x(1)*sind(alpha)^2/(sind(alpha+theta))

limit
%tension

 %%
close all 

xy = [limit.*cosd(sweep)' limit.*sind(sweep)'];

figure
plot(xy(:,1),xy(:,2),'*')
xlabel('Fx')
ylabel('Mz')

figure
plot(sweep,limit)

figure
scatter3(limit3(:,1),limit3(:,2),limit3(:,3))
xlabel('Fx')
ylabel('Fy')
zlabel('Mz')

