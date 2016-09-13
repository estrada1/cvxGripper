%% Plotting multiple rounds of pulloff data
% High level script for generating and plotting pulloff data from datafiles
% Defines where all the files are then calls pulloffParse on each file
addpath('functionsCvx','functionsHelper','dataGenerated')
clear
close all; 
clc; 

showplots = 0; 
offset = 0; 

%directory = 'data/2016_07_22_Pulloff/';
% files = {'straightpull', [directory '01_straightpull'];
%         %'sidepull', [directory '03_sidepull'];
%         'angledpull', [directory '04_angledpull'];
%         %'sidepull2', [directory '05_sidepull'];
%         'angledpull2', [directory '06_angledpull'];
%         'twistup', [directory '07_twistup'];
%         'twistuppull', [directory '08_twistuppull'];
%         'twistdown', [directory '09_twistdown'];
%         'sidepullretake', [directory '10_sidepullretake'];
%         'sidepull4', [directory '11_sidepull4'];
%         'sidepull5', [directory '12_sidepull5'];
%         }


directory = 'data/2016_07_27_PaperPulloff/';
files = {...
        'straight', [directory '02_straight'], ...
            [29.741 47.066 67.159] ;
        'angle', [directory '03_angle'], ...
            [23.495 42.736 58.600] ;
        'twist', [directory '04_twist'], ...
            [22.555 60.342] ;    %39.678 
        %'twist2', [directory '05_twist'], ...
        %           [31.674 44.067 52.725] ;  
            %[30.633 43.715 51.928] ;  

            
        %'side', [directory '08_side'], ...
        %    [74.334] ; 
        'side2', [directory '09_side'], ...
            [44.381, ] ;
            % 48.379
        'side3', [directory '10_side'], ...
            [36.251 57.261 63.477] ; %46.853   
        %'side4', [directory '11_side'], ...
        %    [46.258 56.718 78.941] ; 
        
        % Taking last two from top and putting in middle since it looks
        % like there's a false trend right now
        'side10', [directory '18_side'], ...
            [30.639 47.474] ; 
        'side11', [directory '19_side'], ...
            [21.734 35.342] ; 
            
        'side5', [directory '12_side'], ...
            [34.324 44.046 53.786] ; 
        'torque', [directory '13_torque'], ...
            [33.462 42.143]; %55.137] ;
        'side6', [directory '14_side'], ...
            [34.790 47.032 55.666] ; 
        'side7', [directory '15_side'], ...
            [35.230 43.658 51.557] ; 
        'side8', [directory '16_side'], ...
            [22.158 31.285 40.760] ; 
        % Side 9 is good, just have too much data on one side
        %'side9', [directory '17_side'], ...
        %    [26.138 36.375 45.576] ; 
        
        
        'twist3', [directory '06_twist'], ...
            [28.97 44.521 59.394] ;  
        'angle2', [directory '07_angle'], ...
            [28.936 44.930 53.938] ;   
        }



[mFiles n] = size(files)

for ii = 1:mFiles
    
    thisData = files{ii,2};
    handPicked = files{ii,3}

    limits =...
        importTrials(pulloffParse(thisData,showplots,handPicked),offset);
    data(ii,:) = {files(ii,1), limits}
end

save('dataGenerated/2016_09_11_PaperPulloff','data')
%save('2016_08_08_PureTorque','data')