function [ data ] = importTrials(inputVar,r)
% Take in ATI Data from our experimental setup and translate it to my model
% of the limit surface
% r is ATI offset, measured as a positive number 

ATI_Fz = inputVar(:,4);   % normal to object 
ATI_Fy = inputVar(:,3);   % tangential to object
ATI_Mx = inputVar(:,5);   % moment at base

Model_Ftan = -ATI_Fy; 
Model_Fnor = ATI_Fz; 
Model_Tz = ATI_Mx; 

F2ATI = [1 0 0; 0 1 0; r 0 1];

data = (F2ATI*[Model_Ftan,Model_Fnor,Model_Tz]')';

end