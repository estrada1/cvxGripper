
n = 50; 
a = linspace(-1,1,n)';
b = linspace(0,1,n)';
o = ones(n,1); 
o0 = zeros(n,1); 
on = -ones(n,1); 

A = [o o a; a o o; o b o;...
    on o a; a o on; on b on;...
    o o0 a; a o0 o; o b on;...
    on o0 a; a o0 on; on b o]';

sx = 10;
sy = 4.5;
sz = .15;
    
A = ([sx 0 0; 0 sy 0; 0 0 sz]*A);

R = @(theta) [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];

B = zeros(size(A));


for ii = 1:length(A); 
    B(:,ii) = R(A(3,ii))*A(:,ii); 
end
%     
    figure
    %plot3(A(:,1),A(:,2),A(:,3),'.', 'MarkerSize',10)
    plot3(A(1,:),A(2,:),A(3,:),'.', 'MarkerSize',10)
%     x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-1/2)*sx;
%     y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1])*sy;
%     z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-1/2)*sz;


load('dataGenerated/3DscatterLimit_AsymmetricPaper_Sept8') % limit variable represents as accelerations about object's COM
limit(isinf(limit(:,3)),:) = [];% Get rid of erraneous vals
limit(isnan(limit(:,3)),:) = [];% Get rid of erraneous vals
limitWrist=limit;


figure; set(gca,'fontsize',20); hold on;
    plot3(B(1,:),B(2,:),B(3,:),'.', 'MarkerSize',10)
hold on; 

plotManualIsolines(limitWrist,limitWrist(:,2))
axis tight
%%
figure; hold on; 
plotManualIsolines(B',B(3,:)')


    