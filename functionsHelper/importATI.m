function [ data ] = importATI( filename,r_nor, r_tan)
% Take in ATI Data from our experimental setup and translate it to my model
% of the limit surface
% r is ATI offset, measured as a negative number 

load(filename)

ATI_Fz = ATISensor(:,4);   % normal to object 
ATI_Fy = ATISensor(:,3);   % tangential to object
ATI_Mx = ATISensor(:,5);   % moment at base

Model_Ftan = -ATI_Fy; 
Model_Fnor = ATI_Fz; 
Model_Tz = ATI_Mx; 

F2ATI = [1 0 0; 0 1 0; r_nor r_tan 1];

data = (F2ATI*[Model_Ftan,Model_Fnor,Model_Tz]')';

end