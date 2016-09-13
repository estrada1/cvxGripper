

limitWrist = (trans(r)*limit')';

limit = limitWrist;
nLevels = numel(levels); 
endColor = [165, 3, 12]/255; 
startColor = [140, 213, 255]/255;
dColor = (endColor-startColor)/nLevels; 
figure; set(gca,'fontsize',20); hold on;
for ii = 1:nLevels
    thisFy = levels(ii);
    thisInd = limit(:,2)== thisFy;
    thisLevel = limit(thisInd,:); 
    thisLevel((end/2+1):end,:) = flipud(thisLevel((end/2+1):end,:)); % HACK FOR NOW
    thisLevel = [thisLevel; thisLevel(1,:)]; 
    thisColor = startColor+ii*dColor;
    
    plot3(thisLevel(:,1),thisLevel(:,3),thisLevel(:,2),'LineWidth',6,'Color',thisColor)
end

xlabel('Fx')
ylabel('Mx')    
zlabel('Fy')