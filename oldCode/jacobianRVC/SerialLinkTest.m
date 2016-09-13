close all; clear; 

syms l
subs(l,1)
l = 1
L(1) = Revolute('d', 0, 'a', l, 'alpha', 0);
L(2) = Revolute('d', 0, 'a', l, 'alpha', 0);
L(3) = Revolute('d', 0, 'a', l, 'alpha', 0);



twolink = SerialLink(L, 'name', 'two link');

q = [pi/6 pi/2 -pi/4];

j0 = twolink.jacobn(q)

%twolink.plot(q)

J = j0([1 2 6],:)

Fnet  = 
