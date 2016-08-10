function [ fx, tensions, components ] = cvxGripMaxFx( A, constraints)
% Optimize value for max/min Mz attainable for a given fx anf fy exerted
% ALPHA IS IN DEGREES
% constraints = maximum values allowed for [ t1 t2 c1 c2 ]
% objective dictates whether we maximize or minimize this



 cvx_begin quiet

        variable x(4,1)
        variable fx(1,1)
        variable fy(1,1)
        variable mz(1,1)

        maximize( fx )

        subject to

            x<=constraints
            
            fy>=0

            x>=[0; 0; 0; 0]
            [fx;fy;mz] ==  A * x

    cvx_end
    
    fx_max = cvx_optval;
    tensions = x;
    components = A*x;
    
end

