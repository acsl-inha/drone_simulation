function [thrust, phi_cmd, the_cmd, psi_cmd] = compute_guidance(state, t, t_index)
    global flag;
    global counter;
    global cmd0;
    global cmd1;
    global cmd2;
    global cmd3;
    t_fail = 3;
    t_detect = 0.2;    
    Deg2Rad = pi/180;
    Rad2Deg = 1/Deg2Rad;
    tf = 7;    
    g = 9.8;
    m = 23.56; 
    
    if  flag
        counter = 0;
        cmd0 = 0;
        cmd1 = 0;
        cmd2 = 0;
        cmd3 = 0;
        flag = false;
    end
            
    u = state(1,1);
    v = state(2,1);
    w = state(3,1);
    p = state(4,1);
    q = state(5,1);
    r = state(6,1);
    x = state(7,1);
    y = state(8,1);
    z = state(9,1);
    phi = state(10,1);
    the = state(11,1);
    psi = state(12,1);  % state variables(u,v,w : 기체중심 좌표 / p,q,r : 각속도 / x,y,z : NED 기준 좌표 / phi,the,psi : 자세각 )
    
    s_phi = sin(phi); c_phi = cos(phi);
    s_the = sin(the); c_the = cos(the);
    s_psi = sin(psi); c_psi = cos(psi);
    
    Cbn = ([ ...
        c_psi*c_the, c_psi*s_the*s_phi-s_psi*c_phi, c_psi*s_the*c_phi+s_psi*s_phi; ...
        s_psi*c_the, s_psi*s_the*s_phi+c_psi*c_phi, s_psi*s_the*c_phi-c_psi*s_phi; ...
             -s_the,                   c_the*s_phi,                   c_the*c_phi ...
    ]);
    
    value_dot = Cbn*[u;v;w];
    x_dot = value_dot(1);
    y_dot = value_dot(2);
    z_dot = value_dot(3);
    h_dot = -z_dot;
    h = -z;
    
    omg_alt = 2; zet_alt = 0.8;
    omg_pos = 2; zet_pos = 0.8;
    
    % NED 좌표 기준 위치 명령 h, x, y와 자세각 명령 psi
    if t < t_fail+t_detect
        
        % 고장 전 자세 명령 산출 loop
        h_cmd = 1;
        x_cmd = 1;
        y_cmd = 2;
        psi_cmd = 10*Deg2Rad;
    
        % altitude loop
        up_cmd = -2*zet_alt*omg_alt*h_dot + omg_alt^2*(h_cmd-h) + g;
        thrust = m*up_cmd/(c_phi*c_the);

        % horizontal position loop
        ax_cmd = -2*zet_pos*omg_pos*x_dot + omg_pos^2*(x_cmd-x);
        ay_cmd = -2*zet_pos*omg_pos*y_dot + omg_pos^2*(y_cmd-y);
        value2_dot = Cbn.'*[ax_cmd;ay_cmd;0];
  
        au_cmd = value2_dot(1); 
        av_cmd = value2_dot(2); 
        aw_cmd = value2_dot(3);
        the_cmd = atan2(-au_cmd,g);
        phi_cmd = atan2( av_cmd,g);

    else
        
        % 고장 검출 후 자세 명령 산출 loop(compute_guidance_cvx함수로 계산)    
        if (rem(counter,5)==0)
            position = [x;y;z];
            velocity = [x_dot;y_dot;z_dot];
            [thrust, phi_cmd, the_cmd, psi_cmd] = compute_guidance_cvx( position,velocity,psi,tf-t );
            cmd0 = thrust;
            cmd1 = phi_cmd;
            cmd2 = the_cmd;
            cmd3 = psi_cmd;
        else
            thrust = cmd0;
            phi_cmd = cmd1;
            the_cmd = cmd2;
            psi_cmd = cmd3;
        
        end
        counter = counter + 1;
        
    end
end