function magnitude = solveSurface_polar(fit_polar, forces)
% Matt Estrada
% February 1, 2017 
% Transform the curved gripper limit surface into polar coordinates so that
% one may solve for the magnitude as the output of the surface

x = forces(:,1);
y = forces(:,2);  % Note I switch y and z here 
z = forces(:,3);

[azimuth,elevation,r] = cart2sph(x,y,z);
magnitude = fit_polar([azimuth, elevation]);


end