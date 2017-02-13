clc;
clear;
close all; 

zeta = .4;
wn = 1; 
sigma = zeta*wn; 
wd = wn*sqrt(1-zeta^2); 
x0 = 1; 

dt = wn/1000; 
t = 0:dt:6*wn; 

x = x0*exp(-sigma*t)/wd.*sin(wd*t); 
ddx = diff(diff(x))/(dt^2); 
subplot(2,1,1)
plot(t,x)
ylabel('position')
subplot(2,1,2)
area(t(3:end),ddx)
ylabel('acceleration (aka net force)')
xlabel('time [ms]')