%% Surface Plot At Wrist
addpath('functionsCvx','functionsHelper','dataGenerated')

load('3DmeshLimit')
figure; set(gca,'fontsize',16); hold on 

s = surf(f,m,FyWrist)

xlabel('F_x [N]')
zlabel('F_y [N]')
ylabel('T_z [Nm]')
title('Force/Torque at Wrist')
alpha(s,0.05) % transparency
legend('model')
