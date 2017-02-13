function fit_polar = fitSurface_polar(limitSurfaceData, fitType)
% Matt Estrada
% February 1, 2017 
% Transform the curved gripper limit surface into polar coordinates so that
% one may solve for the magnitude as the output of the surface

x = limitSurfaceData(:,1);
z = limitSurfaceData(:,2);  % Note I switch y and z here 
y = limitSurfaceData(:,3);

[azimuth,elevation,r] = cart2sph(x,y,z);

fit_polar = fit([azimuth, elevation],r,fitType)

end