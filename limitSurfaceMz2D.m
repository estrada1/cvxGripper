function [limit, tensions] = limitSurfaceMz2D(A,constraints, fy)

%% 2D Curved Surface Gripper
% Matt Estrada
% Going off Elliot's drawing in his curved gripper paper
% Doing a 2D slice to calculate result between moment/tangential force
% Super similar to convexGripperPlanar_LimitSurface_Cartesian, just
% ignoring the Fy direction
% June 29 2016
addpath('functionsCvx','functionsHelper','dataGenerated')

%constraints = [maxAdhesion1; maxAdhesion2; 1000000; 1000000];

[ fx_max, tensions, components ] = cvxGripMaxFx( A, constraints);
Fx = [-fx_max:fx_max/25:fx_max]';
nFx = numel(Fx);

objective = 'max';
Mz_max = zeros(size(Fx));
tension_max = zeros(nFx,4);

for nn = 1:nFx
    
    fx = Fx(nn);
    [Mz, vect] = cvxGripMz(A, fx, fy, constraints, objective);
    Mz_max(nn,1) = Mz;
    tension_max(nn,:) = vect;

    disp([ 'Trial ' num2str(nn) ' of ' num2str(2*numel(Fx))])

end

objective = 'min';
Mz_min = zeros(size(Fx));
tension_min = zeros(nFx,4);

for nn = 1:nFx
    
    fx = Fx(nFx-nn+1); % Sweep back the opposite directon to avoid weird plotting
    [Mz, vect] = cvxGripMz(A, fx, fy, constraints, objective);
    Mz_min(nn,1) = Mz;
    tension_min(nn,:) = vect;

    disp([ 'Trial ' num2str(nn+nFx) ' of ' num2str(2*nFx)])

end
Fy = fy*ones(nFx,1);

limit = [Fx Fy Mz_max ; flipud(Fx) Fy Mz_min]; % Make sure to reverse Fx on second entry here 
tensions = [tension_max; tension_min];

end

