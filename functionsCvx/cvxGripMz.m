function [ Mz, vect ] = cvxGripMz( A, fx, fy, constraints, objective )
% Optimize value for max/min Mz attainable for a given fx anf fy exerted
% constraints = maximum values allowed for [ t1 t2 c1 c2 ]
% objective dictates whether we maximize or minimize this

 cvx_begin quiet

        variable x(4,1)
        variable mz(1,1)

        %fx = Fx(nn);
        % fy = 0;

        if strcmp(objective,'max')
            maximize( mz )
        end
        
        if strcmp(objective,'min')
             minimize( mz )
        end


        subject to

            x<=constraints

            x>=[0; 0; 0; 0]
            [fx fy mz]' ==  A * x

    cvx_end
    
    Mz = cvx_optval; 
    vect = x;
end

