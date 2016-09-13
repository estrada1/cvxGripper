function plotManualIsolines(data,grouping)
%% Plot data in a more inuitive form-- connecting isolines. 
% Matt Estrada
% Aug 30 2016
% Tried plotting isolines already but it didn't connect the ends, which
% sucked 

    % Plot on isoline "levels" 
    levels = unique(grouping);
    nLevels = numel(levels)
    
    % Gradually fade colors to indicate Fy value
    endColor = [165, 3, 12]/255; 
    startColor = [140, 213, 255]/255;
    dColor = (endColor-startColor)/nLevels; 
    
    for ii = 1:nLevels
        thisFy = levels(ii);
        thisInd = grouping == thisFy;
        thisLevel = data(thisInd,:); 
        %thisLevel((end/2+1):end,:) = flipud(thisLevel((end/2+1):end,:)); % HACK FOR NOW
        thisLevel = [thisLevel; thisLevel(1,:)]; 
        thisColor = startColor+ii*dColor;
        plot3(thisLevel(:,1),thisLevel(:,3),thisLevel(:,2),'LineWidth',6,'Color',thisColor)
        %plot3(thisLevel(:,1),thisLevel(:,2),thisLevel(:,3),'LineWidth',6,'Color',thisColor)
        xlabel('F_x [N]')
        ylabel('M_z [Nm]')    
        zlabel('F_y [N]')
    end

end
