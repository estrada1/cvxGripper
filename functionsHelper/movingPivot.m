function [ correctedData ] = movingPivot( data, k)
%% Function to correct the fact that the pivot is translating across the normal axis
% data - already corrected for the nominal (resting) offset of the gripper
% k - linear stiffness of the normal axis

trans = @(r) [1 0 0; 0 1 0; r 0 1];

[m,n] = size(data);
correctedData = zeros(m,n); 

for nn = 1:m
    dy = data(nn,2)/k; 
    correctedData(nn,:) = (trans(-dy)*data(nn,:)')';
end

end