function [R, dRdU, dRdf] = RESFUN_el2el(Uf, K, Fv, Lc, Gc, knl)
    fnl_pred = knl*Lc*Uf(1:end-1);	% Predicted normal force
    fnl = max(fnl_pred, 0);		% Saturated @ 0: unilateral spring

    jnl = knl*Lc;			% Jacobian
    jnl(fnl==0,:) = 0;			

    % Static Residue
    R = K*Uf(1:end-1)+Gc*fnl - Fv*Uf(end);
    dRdU = K+Gc*Lc*knl;
    dRdf = -Fv;
end
