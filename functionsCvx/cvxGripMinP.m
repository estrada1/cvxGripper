function [ minP, Fnet, tensions, components ] = cvxGripMinP( A, constraints, v)
% Optimize value for max/min Mz attainable for a given fx anf fy exerted
% ALPHA IS IN DEGREES
% constraints = maximum values allowed for [ t1 t2 c1 c2 ]
% objective dictates whether we maximize or minimize this



 cvx_begin quiet

        variable x(4,1)
        variable fx(1,1)
        variable fy(1,1)
        variable mz(1,1)
        variable fnet(3,1)

        minimize( fnet'*v )

        subject to

            x<=constraints
            
            fy>=0

            x>=[0; 0; 0; 0]
            
            fnet == [fx;fy;mz]
            
            fnet ==  A * x

    cvx_end
    
    minP = cvx_optval;
    tensions = x;
    components = A*x;
    Fnet = fnet;
    
end

