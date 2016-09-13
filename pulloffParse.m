function [trials] = pulloffParse(filename,showplots,handPicked)

addpath('functionsCvx','functionsHelper','dataGenerated')

    % Plotting ATI Trials to measure force at adhesive failure
    % Bessie and Matt 
    % July 2016

    % Load in data
    
    load(filename);

    disp(['Parsing file: ' filename])
    if ~isempty(handPicked)
        disp('Using handpicked maximums')
        nPicked = length(handPicked); 
        for jj = 1:nPicked
            thisPick = handPicked(jj); 
            indPicked = thisPick*1000-1; % Assuming ATI is recording in ms
            trials(jj,:) = ATISensor(indPicked,:); 
        end
    else
        %%
        % need to be able to filter the tests without having to stop each
        % time...somehow be able to read a tap that would indicate the start of a
        % new test
        [row,column] = size(ATISensor);
        index = 1;
        i = 1;
        while i <= row
            if ATISensor(i,2) < -5
                tap_array(index) = i;
                index = index + 1;
                i = i + 6000;
            end
            i = i + 1;
        end
        disp([num2str(length(tap_array)) ' taps detected'])
        % For loop that finds the limits of each trial and stores them in a matrix

        [x,trial_count] = size(tap_array);

        for j = 1:trial_count
            % initialize the time intervals based on taps
            t1 = tap_array(j);
            if j == trial_count
                t2 = row;
            else
                t2 = tap_array(j+1);
            end

            % Script to determine the limit of the force magnitude
            F_mag = (ATISensor([t1:t2],3)).^2 + (ATISensor([t1:t2],4)).^2;    % magnitude squared
            % Note: ignore x force since taps can interfere with this detection
                % x force square was ATISensor([t1:t2],2)).^2
            [F_mag_max,ind_max] = max(F_mag);

                %%% what we are trying to find%%%
                limit = ATISensor(t1+ind_max-1,:);

            % add the array of limits of forces and magnatudes to trials matrix   
            trials(j,:) = limit(1,:);

        end
    end

    %% Plots
    if showplots
        time = ATISensor(:,1);

        figure
        hold on
        plot(time,ATISensor(:,2),time,ATISensor(:,3),time,ATISensor(:,4))

        plot(trials(:,1),trials(:,2),'kx','MarkerSize',10,'LineWidth',3)
        plot(trials(:,1),trials(:,3),'kx','MarkerSize',10,'LineWidth',3)
        plot(trials(:,1),trials(:,4),'kx','MarkerSize',10,'LineWidth',3)
        if nargin < 3
            plot(time(tap_array),ATISensor(tap_array,2),'ko','MarkerSize',10,'LineWidth',2)
        end
        legend('Fx','Fy','Fz','max force','tap')
        title(filename)
        
        %k = waitforbuttonpress;
        
        % 
        % figure
        % plot(ATISensor(:,1),ATISensor(:,5),ATISensor(:,1),ATISensor(:,6),ATISensor(:,1),ATISensor(:,7))
        % legend('Mx','My','Mz')
        % 
        % figure
        % scatter3(trials(:,3),trials(:,4),trials(:,5));
        % xlabel('Fy'); % ATI Coordinates
        % ylabel('Fz');
        % zlabel('Mx');
    end
end