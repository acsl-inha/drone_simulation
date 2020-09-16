function  [thrust,phi_cmd,the_cmd,psi_cmd]= compute_guidance_cvx(position,velocity,psi,tgo)
%% initial value

    delt = 0.05;
    T = tgo;
    N = int(T/delt);
    
    r_des = [1; 1; 0];
    v_des = [0; 0; 0];
    g_pin = [0; 0; 9.8];
    T2 = 720;
    
  
 %% cvx_algorithm
 
 
    cvx_solver mosek
    cvx_begin 
       
        variable T_c(3,N+1);
        variable r_pin(3,N+1);
        variable v_pin(3,N+1);
        
        mag = norm(T_c); 
        
        minimize (norm(mag));
        
        subject to
            v_pin(:,0) == velocity;
            r_pin(:,0) == position;
            v_pin(:,end) == v_des;
            r_pin(:,end) == r_des;
        
            for t = 1:N
                
                r_pin(:,t+1) == r_pin(:,t) + v_pin(:,t)*delt;
                v_pin(:,t+1) == v_pin(:,t) + (T_c(:,t)/m + g_pin)*delt;
                norm(T_c(:,t)) <= T2;
                T_c(2,t) <= 0

            end
            
    cvx_end
    
    if cvx_status == 'Solved'
        [T_n,T_e,T_d] = T_c(:,0);
        T_a = T_n*cos(psi) + T_e*sin(psi);
        T_b = T_n*sin(psi) + T_e*cos(psi);
        thrust = sqrt(T_d^2 + T_n^2 + T_e^2);
        the_cmd = -atan2(T_a,-T_d);
        phi_cmd = asin(T_b/thrust);
        psi_cmd = psi;
    
    else
        thrust = 0;
        phi_cmd = 0;
        the_cmd = 0;
        psi_cmd] = 0;
    end
    
    if position(3) >= 0
        thrust = 0;
        phi_cmd = 0;
        the_cmd = 0;
        psi_cmd] = 0;
    end
    
end
