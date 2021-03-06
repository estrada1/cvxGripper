function [ beta, unit_vect, components ] = cvxGripBeta( A, d, constraints)
% Optimize value for max/min Mz attainable for a given fx anf fy exerted
% ALPHA IS IN DEGREES
% constraints = maximum values allowed for [ t1 t2 c1 c2 ]
% d is "direction" of tug, or constrained proportions of fx:fy:mz
% objective dictates whether we maximize or minimize this

 cvx_begin quiet

        variable x(4,1)
        variable fnet(1,1)

        maximize( fnet )

        subject to

            x<=constraints

            x>=[0; 0; 0; 0]
            fnet*d ==  A * x

    cvx_end
    
    beta = cvx_optval;
    unit_vect = x;
    components = A*x;
    
end

