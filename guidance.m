function chi_d = guidance(pos, x_t, y_t, x_ref, y_ref, R_los)
    x = pos(1); y = pos(2);
    y_e = crosstrackWpt(x_t, y_t, x_ref, y_ref, x, y);
%     lookahead = sqrt(R_los^2-y_e^2);
    lookahead = 800;
    K_p = 1/lookahead;
    pi_p = atan2(y_t-y_ref, x_t-x_ref);
    chi_d = pi_p - atan(K_p*y_e);
end

