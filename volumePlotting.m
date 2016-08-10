close all; clear; clc;
%load('3DscatterLimit_paperAsymmetric');

trans = @(d)[1 0 0; 0 1 0; d 0 1];

alpha = 11.35;      % [deg]
r = 9/2*0.0254;     % [m]

constraints = [20 20 10e10 10e10]';
A = defineGeometry(alpha,r);

Awrist = trans(r)*A;

[ mz_max, tensions, components ] = cvxGripMaxMz( Awrist, constraints);
[ fx_max, tensions, components ] = cvxGripMaxFx( Awrist, constraints);
res = 10; 
f = -fx_max:fx_max/res:fx_max'; 
m = -mz_max:mz_max/res:mz_max'; 

Fy = zeros(numel(f),numel(m));
nTrials = numel(Fy);
disp(['Total trials: ' num2str(nTrials)])

for ii = 1:numel(f)
    
    fx = f(ii);

    for jj = 1:numel(m)
        
        mz = m(jj);
        
        trial = (ii-1)*numel(f) + jj; 
        disp(['Trial : ' num2str(trial) ' of ' num2str(nTrials)])
        
        [ fy, vect ] = cvxGripFy( Awrist, fx, mz, constraints );
        
        Fy(ii,jj) = fy; 
        
    end
    
end

Fy
%%
figure; set(gca,'fontsize',16); hold on 
mesh(f,m,Fy)
xlabel('F_x [N]')
zlabel('F_y [N]')
ylabel('T_z [Nm]')
title('Force/Torque at Wrist')
        
vector = zeros(numel(Fy),3); 
%%
figure
surf(f,m,Fy)
%tri = delaunay(mesh_result(1, :), mesh_result(2, :));
%trisurf(tri, mesh_result(1, :), mesh_result(2, :), -mesh_result(3, :))

%[X,Y] = meshgrid(xgv,ygv)
