
function rho = ComputeInverseAngleAxis(R1, R2)
   
    err_thr= 0.0001; 
    
    % inverse-equivalent angle-axis algorithm to compute the misalignment
    % vector
    cos_th= 1/2*(sum(sum(R1.*R2)) - 1);
    
    if (cos_th >= (1 - err_thr))
        % Case when cos(th) is almost 1 (no rotation, th= 0)
        rho= [0 ; 0 ; 0];
        
    elseif (abs(cos_th) < (1 - err_thr))
        % Case when abs(cos(th)) smaller then 1
        v_times_sin_th= 1/2*(sum(cross(R1, R2, 1)));
        sin_th= norm(v_times_sin_th);
        
        th= atan2(sin_th, cos_th);
        v= v_times_sin_th/sin_th;
        
        rho= (v*th)';
        
    else
        % Case when cos(th) is almost -1 (flip around a single axis th= pi)
        R= R1 + R2;
        th= pi;
        v= zeros(3, 1);
        for i= 1:3
            col_i= R(:, i);
            if( abs(col_i(1)) > err_thr || ...
                abs(col_i(2)) > err_thr || ...
                abs(col_i(3)) > err_thr  )  
           
               % choose as axis of rotation the non-null column
               v= col_i;
               % normalize the vector
               v= v/norm(v);
            end
        end
       
        rho= v*th;
    end
end

