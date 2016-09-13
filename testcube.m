clear; clc; close all; 

figure
sx = 3.2;
sy = 1.3
sz = 2.0;
x=[0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]*sx;
y=[0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]*sy;
z=[0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]*sz;
for i=1:6
    h=patch(x(:,i),y(:,i),z(:,i),[0 0 0.3]);
    set(h,'edgecolor','k','FaceAlpha',0.2)
end

