function [t,VAR,Output] = PassiveActiveGripper
%===========================================================================
% File: PassiveActiveGripper.m created on Sun Sep 11 2016 by MotionGenesis 5.8.
% Portions copyright (c) 2009-2016 Motion Genesis LLC.  Rights reserved.
% Advanced Student Licensee: Matt Estrada (until January 2019).
% Paid-up MotionGenesis Advanced Student licensees are granted the right
% right to distribute this code for legal student-academic (non-professional) purposes only,
% provided this copyright notice appears in all copies and distributions.
%===========================================================================
% The software is provided "as is", without warranty of any kind, express or    
% implied, including but not limited to the warranties of merchantability or    
% fitness for a particular purpose. In no event shall the authors, contributors,
% or copyright holders be liable for any claim, damages or other liability,     
% whether in an action of contract, tort, or otherwise, arising from, out of, or
% in connection with the software or the use or other dealings in the software. 
%===========================================================================
eventDetectedByIntegratorTerminate1OrContinue0 = [];
fx=0; fy=0; mz=0;
xpTarB=0; ypTarB=0; thetapp=0; xpp=0; ypp=0; KineticEnergy=0;

addpath('functionsCvx','functionsHelper','dataGenerated')
trans = @(rd) [1 0 0; 0 1 0; rd 0 1];
% Define Geometry 
alphad = 11.35;         % [deg]
r = 9/2*0.0254;         % [m]
Acm = defineGeometry(alphad,r); 
Awrist = trans(r)*Acm; 
A = Awrist; 

maxAdhesion1 =  24;
maxAdhesion2 = 19.28;
constraints = [maxAdhesion1; maxAdhesion2; 100; 100];


%-------------------------------+--------------------------+-------------------+-----------------
% Quantity                      | Value                    | Units             | Description
%-------------------------------|--------------------------|-------------------|-----------------
ITarzz                          =  1.5128;                  % kg*m^2              Constant
mTar                            =  10.5526;                 % kg                  Constant
r                               =  0.1143;                 % m                   Constant

theta                           =  0;                      % rad                 Initial Value
x                               =  0;                   % m                   Initial Value
y                               =  0;                      % m                   Initial Value
thetap                          =  0;                      % rad/sec             Initial Value
xp                              =  0.3;                    % m/s                 Initial Value
yp                              =  -1.3;                    % m/s                 Initial Value

tInitial                        =  0.0;                    % second              Initial Time
tFinal                          =  .1;                     % sec                 Final Time
tStep                           =  0.02;                  % sec                 Integration Step
printIntScreen                  =  1;                      % 0 or +integer       0 is NO screen output
printIntFile                    =  1;                      % 0 or +integer       0 is NO file   output
absError                        =  1.0E-06;                %                     Absolute Error
relError                        =  1.0E-06;                %                     Relative Error
%-------------------------------+--------------------------+-------------------+-----------------


VAR = SetMatrixFromNamedQuantities;
[t,VAR,Output] = IntegrateForwardOrBackward( tInitial, tFinal, tStep, absError, relError, VAR, printIntScreen, printIntFile );
OutputToScreenOrFile( [], 0, 0 );   % Close output files
PlotOutputFiles;


%===========================================================================
function sys = mdlDerivatives( t, VAR, uSimulink )
%===========================================================================
SetNamedQuantitiesFromMatrix( VAR );

% Quantities to be specified (not assigned in MotionGenesis).
fx = 0;
fy = 0;
mz = 0;

epsilon = .0001; 
t
KineticEnergy

% Min Power
% [ minP, Fnet, tensions, components ] = cvxGripMinP( A, constraints,...
%     [xpTarB + sign(xpTarB)*epsilon; ypTarB + sign(ypTarB)*epsilon; thetap + sign(thetap)*epsilon]);
% Fnet'
% fx = Fnet(1); fy = Fnet(2); mz = Fnet(3); 

% Oppose Vel
d = [-xpTarB/(abs(xpTarB) +epsilon); -ypTarB/(abs(ypTarB) +epsilon); -thetap/(abs(thetap) +epsilon)];
[ beta, unit_vect, components ] = cvxGripBeta( A, d, constraints);
fx = beta*d(1); fy = beta*d(2); mz = beta*d(3); 
Fnet = [fx fy mz]

% Sanity Check 
%fx = 1; fy = 1; mz = 1; 


xpp = fx/mTar;
ypp = fy/mTar;
thetapp = (mz-r*fx*cos(theta)-r*fy*sin(theta))/ITarzz;

sys = transpose( SetMatrixOfDerivativesPriorToIntegrationStep );
end



%===========================================================================
function VAR = SetMatrixFromNamedQuantities
%===========================================================================
VAR = zeros( 1, 6 );
VAR(1) = theta;
VAR(2) = x;
VAR(3) = y;
VAR(4) = thetap;
VAR(5) = xp;
VAR(6) = yp;
end


%===========================================================================
function SetNamedQuantitiesFromMatrix( VAR )
%===========================================================================
theta = VAR(1);
x = VAR(2);
y = VAR(3);
thetap = VAR(4);
xp = VAR(5);
yp = VAR(6);
end


%===========================================================================
function VARp = SetMatrixOfDerivativesPriorToIntegrationStep
%===========================================================================
VARp = zeros( 1, 6 );
VARp(1) = thetap;
VARp(2) = xp;
VARp(3) = yp;
VARp(4) = thetapp;
VARp(5) = xpp;
VARp(6) = ypp;
end



%===========================================================================
function Output = mdlOutputs( t, VAR, uSimulink )
%===========================================================================
xpTarB = sin(theta)*yp + cos(theta)*xp - r*thetap;
ypTarB = cos(theta)*yp - sin(theta)*xp;
KineticEnergy = 0.5*ITarzz*thetap^2 + 0.5*mTar*(xp^2+yp^2);

Output = zeros( 1, 14 );
Output(1) = t;
Output(2) = x;
Output(3) = y;

Output(4) = t;
Output(5) = theta;

Output(6) = t;
Output(7) = fx;
Output(8) = fy;
Output(9) = mz;

Output(10) = t;
Output(11) = xpTarB;
Output(12) = ypTarB;

Output(13) = t;
Output(14) = KineticEnergy;
end


%===========================================================================
function OutputToScreenOrFile( Output, shouldPrintToScreen, shouldPrintToFile )
%===========================================================================
persistent FileIdentifier hasHeaderInformationBeenWritten;

if( isempty(Output) ),
   if( ~isempty(FileIdentifier) ),
      for( i = 1 : 5 ),  fclose( FileIdentifier(i) );  end
      clear FileIdentifier;
      fprintf( 1, '\n Output is in the files PassiveActiveGripper.i  (i=1, ..., 5)\n\n' );
   end
   clear hasHeaderInformationBeenWritten;
   return;
end

if( isempty(hasHeaderInformationBeenWritten) ),
   if( shouldPrintToScreen ),
      fprintf( 1,                '%%       t              x              y\n' );
      fprintf( 1,                '%%     (sec)           (m)            (m)\n\n' );
   end
   if( shouldPrintToFile && isempty(FileIdentifier) ),
      FileIdentifier = zeros( 1, 5 );
      FileIdentifier(1) = fopen('PassiveActiveGripper.1', 'wt');   if( FileIdentifier(1) == -1 ), error('Error: unable to open file PassiveActiveGripper.1'); end
      fprintf(FileIdentifier(1), '%% FILE: PassiveActiveGripper.1\n%%\n' );
      fprintf(FileIdentifier(1), '%%       t              x              y\n' );
      fprintf(FileIdentifier(1), '%%     (sec)           (m)            (m)\n\n' );
      FileIdentifier(2) = fopen('PassiveActiveGripper.2', 'wt');   if( FileIdentifier(2) == -1 ), error('Error: unable to open file PassiveActiveGripper.2'); end
      fprintf(FileIdentifier(2), '%% FILE: PassiveActiveGripper.2\n%%\n' );
      fprintf(FileIdentifier(2), '%%       t            theta\n' );
      fprintf(FileIdentifier(2), '%%     (sec)          (rad)\n\n' );
      FileIdentifier(3) = fopen('PassiveActiveGripper.3', 'wt');   if( FileIdentifier(3) == -1 ), error('Error: unable to open file PassiveActiveGripper.3'); end
      fprintf(FileIdentifier(3), '%% FILE: PassiveActiveGripper.3\n%%\n' );
      fprintf(FileIdentifier(3), '%%       t             fx             fy             mz\n' );
      fprintf(FileIdentifier(3), '%%     (sec)           (N)            (N)           (N*m)\n\n' );
      FileIdentifier(4) = fopen('PassiveActiveGripper.4', 'wt');   if( FileIdentifier(4) == -1 ), error('Error: unable to open file PassiveActiveGripper.4'); end
      fprintf(FileIdentifier(4), '%% FILE: PassiveActiveGripper.4\n%%\n' );
      fprintf(FileIdentifier(4), '%%       t           xpTarB         ypTarB\n' );
      fprintf(FileIdentifier(4), '%%     (sec)          (m/s)          (m/s)\n\n' );
      FileIdentifier(5) = fopen('PassiveActiveGripper.5', 'wt');   if( FileIdentifier(5) == -1 ), error('Error: unable to open file PassiveActiveGripper.5'); end
      fprintf(FileIdentifier(5), '%% FILE: PassiveActiveGripper.5\n%%\n' );
      fprintf(FileIdentifier(5), '%%       t        KineticEnergy\n' );
      fprintf(FileIdentifier(5), '%%     (sec)         (UNITS)\n\n' );
   end
   hasHeaderInformationBeenWritten = 1;
end

if( shouldPrintToScreen ), WriteNumericalData( 1,                 Output(1:3) );  end
if( shouldPrintToFile ),   WriteNumericalData( FileIdentifier(1), Output(1:3) );  end
if( shouldPrintToFile ),   WriteNumericalData( FileIdentifier(2), Output(4:5) );  end
if( shouldPrintToFile ),   WriteNumericalData( FileIdentifier(3), Output(6:9) );  end
if( shouldPrintToFile ),   WriteNumericalData( FileIdentifier(4), Output(10:12) );  end
if( shouldPrintToFile ),   WriteNumericalData( FileIdentifier(5), Output(13:14) );  end
end


%===========================================================================
function WriteNumericalData( fileIdentifier, Output )
%===========================================================================
numberOfOutputQuantities = length( Output );
if( numberOfOutputQuantities > 0 ),
   for( i = 1 : numberOfOutputQuantities ),
      fprintf( fileIdentifier, ' %- 14.6E', Output(i) );
   end
   fprintf( fileIdentifier, '\n' );
end
end



%===========================================================================
function PlotOutputFiles
%===========================================================================
if( printIntFile == 0 ),  return;  end
figure;
data = load( 'PassiveActiveGripper.1' ); 
plot( data(:,1),data(:,2),'-b', data(:,1),data(:,3),'-.g', 'LineWidth',3 );
legend( 'x (m)', 'y (m)' );
xlabel('t (sec)');   % ylabel('Some y-axis label');   title('Some plot title');
clear data;

figure;
data = load( 'PassiveActiveGripper.2' ); 
plot( data(:,1),data(:,2),'-b', 'LineWidth',3 );
legend( 'theta (rad)' );
xlabel('t (sec)');   ylabel('theta (rad)');   % title('Some plot title');
clear data;

figure;
data = load( 'PassiveActiveGripper.3' ); 
plot( data(:,1),data(:,2),'-b', data(:,1),data(:,3),'-.g', data(:,1),data(:,4),'--r', 'LineWidth',3 );
legend( 'fx (N)', 'fy (N)', 'mz (N*m)' );
xlabel('t (sec)');   % ylabel('Some y-axis label');   title('Some plot title');
clear data;

figure;
data = load( 'PassiveActiveGripper.4' ); 
plot( data(:,1),data(:,2),'-b', data(:,1),data(:,3),'-.g', 'LineWidth',3 );
legend( 'xpTarB (m/s)', 'ypTarB (m/s)' );
xlabel('t (sec)');   % ylabel('Some y-axis label');   title('Some plot title');
clear data;

figure;
data = load( 'PassiveActiveGripper.5' ); 
plot( data(:,1),data(:,2),'-b', 'LineWidth',3 );
legend( 'KineticEnergy' );
xlabel('t (sec)');   ylabel('KineticEnergy ');   % title('Some plot title');
clear data;
end



%===========================================================================
function [functionsToEvaluateForEvent, eventTerminatesIntegration1Otherwise0ToContinue, eventDirection_AscendingIs1_CrossingIs0_DescendingIsNegative1] = EventDetection( t, VAR, uSimulink )
%===========================================================================
% Detects when designated functions are zero or cross zero with positive or negative slope.
% Step 1: Uncomment call to mdlDerivatives and mdlOutputs.
% Step 2: Change functionsToEvaluateForEvent,                      e.g., change  []  to  [t - 5.67]  to stop at t = 5.67.
% Step 3: Change eventTerminatesIntegration1Otherwise0ToContinue,  e.g., change  []  to  [1]  to stop integrating.
% Step 4: Change eventDirection_AscendingIs1_CrossingIs0_DescendingIsNegative1,  e.g., change  []  to  [1].
% Step 5: Possibly modify function EventDetectedByIntegrator (if eventTerminatesIntegration1Otherwise0ToContinue is 0).
%---------------------------------------------------------------------------
% mdlDerivatives( t, VAR, uSimulink );        % UNCOMMENT FOR EVENT HANDLING
 mdlOutputs(     t, VAR, uSimulink );        % UNCOMMENT FOR EVENT HANDLING
functionsToEvaluateForEvent = [KineticEnergy-.5];
eventTerminatesIntegration1Otherwise0ToContinue = [1];
eventDirection_AscendingIs1_CrossingIs0_DescendingIsNegative1 = [];
eventDetectedByIntegratorTerminate1OrContinue0 = eventTerminatesIntegration1Otherwise0ToContinue;
end


%===========================================================================
function [isIntegrationFinished, VAR] = EventDetectedByIntegrator( t, VAR, nIndexOfEvents )
%===========================================================================
isIntegrationFinished = eventDetectedByIntegratorTerminate1OrContinue0( nIndexOfEvents );
if( ~isIntegrationFinished ),
   SetNamedQuantitiesFromMatrix( VAR );
%  Put code here to modify how integration continues.
   VAR = SetMatrixFromNamedQuantities;
end
end



%===========================================================================
function [t,VAR,Output] = IntegrateForwardOrBackward( tInitial, tFinal, tStep, absError, relError, VAR, printIntScreen, printIntFile )
%===========================================================================
OdeMatlabOptions = odeset( 'RelTol',relError, 'AbsTol',absError, 'MaxStep',tStep, 'Events',@EventDetection );
t = tInitial;                 epsilonT = 0.001*tStep;                   tFinalMinusEpsilonT = tFinal - epsilonT;
printCounterScreen = 0;       integrateForward = tFinal >= tInitial;    tAtEndOfIntegrationStep = t + tStep;
printCounterFile   = 0;       isIntegrationFinished = 0;
mdlDerivatives( t, VAR, 0 );
while 1,
   if( (integrateForward && t >= tFinalMinusEpsilonT) || (~integrateForward && t <= tFinalMinusEpsilonT) ), isIntegrationFinished = 1;  end
   shouldPrintToScreen = printIntScreen && ( isIntegrationFinished || printCounterScreen <= 0.01 );
   shouldPrintToFile   = printIntFile   && ( isIntegrationFinished || printCounterFile   <= 0.01 );
   if( isIntegrationFinished || shouldPrintToScreen || shouldPrintToFile ),
      Output = mdlOutputs( t, VAR, 0 );
      OutputToScreenOrFile( Output, shouldPrintToScreen, shouldPrintToFile );
      if( isIntegrationFinished ), break;  end
      if( shouldPrintToScreen ), printCounterScreen = printIntScreen;  end
      if( shouldPrintToFile ),   printCounterFile   = printIntFile;    end
   end
   [TimeOdeArray, VarOdeArray, timeEventOccurredInIntegrationStep, nStatesArraysAtEvent, nIndexOfEvents] = ode45( @mdlDerivatives, [t tAtEndOfIntegrationStep], VAR, OdeMatlabOptions, 0 );
   if( isempty(timeEventOccurredInIntegrationStep) ),
      lastIndex = length( TimeOdeArray );
      t = TimeOdeArray( lastIndex );
      VAR = VarOdeArray( lastIndex, : );
      printCounterScreen = printCounterScreen - 1;
      printCounterFile   = printCounterFile   - 1;
      if( abs(tAtEndOfIntegrationStep - t) >= abs(epsilonT) ), warning('numerical integration failed'); break;  end
      tAtEndOfIntegrationStep = t + tStep;
      if( (integrateForward && tAtEndOfIntegrationStep > tFinal) || (~integrateForward && tAtEndOfIntegrationStep < tFinal) ) tAtEndOfIntegrationStep = tFinal;  end
   else
      t = timeEventOccurredInIntegrationStep( 1 );    % time  at firstEvent = 1 during this integration step.
      VAR = nStatesArraysAtEvent( 1, : );             % state at firstEvent = 1 during this integration step.
      printCounterScreen = 0;
      printCounterFile   = 0;
      [isIntegrationFinished, VAR] = EventDetectedByIntegrator( t, VAR, nIndexOfEvents(1) );
   end
end
end


%=============================================
end    % End of function PassiveActiveGripper
%=============================================
