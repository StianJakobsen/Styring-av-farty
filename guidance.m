function chi_d = guidance(pos, x_t, y_t, x_ref, y_ref, h)
    persistent y_e_int;
    if isempty(y_e_int)
        y_e_int = 0;
    end
    x = pos(1); y = pos(2);
    y_e = crosstrackWpt(x_t, y_t, x_ref, y_ref, x, y);
%     lookahead = sqrt(R_los^2-y_e^2);
    lookahead = 800;
    kappa = 0.01;          %Set to 0 for normal LOS
    K_p = 1/lookahead;
    K_i = kappa*K_p;
    
    pi_p = atan2(y_t-y_ref, x_t-x_ref);
    
    ye_int_dot = (lookahead*y_e)/(lookahead^2 + (y_e+ kappa*y_e_int)^2);
    y_e_int = euler2(ye_int_dot, y_e_int, h);
     
    chi_d = pi_p - atan(K_p*y_e + K_i*y_e_int);
end

