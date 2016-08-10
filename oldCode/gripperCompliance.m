%% gripperCompliance
% Calculating and visualizing how stiff I should make this gripper's wrist
% (for the actual case I'm using in experiments)
% Author: Matt Estrada
% Date started: 7/5/16
%% Geometry and FBD 
d = 9*0.0254;    % [m] Object diameter, (which was 9 inches)
w = .045;       % [m]
alpha = asin(w/(d))*180/pi   % [deg] Angle at which contact makes with surface
theta = 0;      % [rad] pulling straight away from gripper 
F = 21;         % [N] pull-off test, straight out
T = sind(theta+alpha)*F/(2*sind(alpha)^2)

