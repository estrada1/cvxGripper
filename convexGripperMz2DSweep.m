function [limit, tensions] = convexGripperMz2DSweep(A,maxAdhesion)

%% 2D Curved Surface Gripper
% Matt Estrada
% Going off Elliot's drawing in his curved gripper paper
% Doing a 2D slice to calculate result between moment/tangential force
% Super similar to convexGripperPlanar_LimitSurface_Cartesian, just
% ignoring the Fy direction
% June 29 2016

constraints = [maxAdhesion; maxAdhesion; 1000000; 1000000];

[ fx_max, tensions, components ] = cvxGripMaxFx( A, constraints);
Fx = [-fx_max:fx_max/50:fx_max]';
nFx = numel(Fx);

fy = 0;

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
    
    fx = Fx(nn);
    [Mz, vect] = cvxGripMz(A, fx, fy, constraints, objective);
    Mz_min(nn,1) = Mz;
    tension_min(nn,:) = vect;

    disp([ 'Trial ' num2str(nn+numel(Fx)) ' of ' num2str(2*numel(Fx))])

end
Fy = zeros(nFx,1);

limit = [Fx Fy Mz_max ; Fx Fy Mz_min];
tensions = [tension_max; tension_min];

end

