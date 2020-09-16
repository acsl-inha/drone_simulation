function [thrust, phi_cmd, the_cmd, psi_cmd] = compute_guidance(state, t, t_index)
    if  ~isprop(compute_guidance, "counter")
         compute_guidance.counter = 0;
         compute_guidance.cmd0 = 0;
         compute_guidance.cmd1 = 0;
         compute_guidance.cmd2 = 0;
         compute_guidance.cmd3 = 0;
    end
            
    u,v,w,p,q,r,x,y,z,phi,the,psi = state(1:12);  % state variables(u,v,w : 기체중심 좌표 / p,q,r : 각속도 / x,y,z : NED 기준 좌표 / phi,the,psi : 자세각 )
    
    s_phi = sin(phi); c_phi = cos(phi);
    s_the = sin(the); c_the = cos(the);
    s_psi = sin(psi); c_psi = cos(psi);
    
    Cbn = ([ ...
        [c_psi*c_the, c_psi*s_the*s_phi-s_psi*c_phi, c_psi*s_the*c_phi+s_psi*s_phi], ...
        [s_psi*c_the, s_psi*s_the*s_phi+c_psi*c_phi, s_psi*s_the*c_phi-c_psi*s_phi], ...
        [     -s_the,                   c_the*s_phi,                   c_the*c_phi] ...
    ]);

    x_dot,y_dot,z_dot = dot(Cbn,[u,v,w]);
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
        [au_cmd, av_cmd, aw_cmd] = dot(Cbn.',[ax_cmd,ay_cmd,0]);
        the_cmd = atan2(-au_cmd,g);
        phi_cmd = atan2( av_cmd,g);

    else
        
        % 고장 검출 후 자세 명령 산출 loop(compute_guidance_cvx함수로 계산)
        
        if (rem(compute_guidance.counter,5)==0)
            [thrust, phi_cmd, the_cmd, psi_cmd] = compute_guidance_cvx([x,y,z],[x_dot,y_dot,z_dot],psi,tf-t);
            compute_guidance.cmd0 = thrust;
            compute_guidance.cmd1 = phi_cmd;
            compute_guidance.cmd2 = the_cmd;
            compute_guidance.cmd3 = psi_cmd;
        else
            thrust = compute_guidance.cmd0;
            phi_cmd = compute_guidance.cmd1;
            the_cmd = compute_guidance.cmd2;
            psi_cmd = compute_guidance.cmd3;
        
        compute_guidance.counter = compute_guidance.counter + 1;

        end
    end
end