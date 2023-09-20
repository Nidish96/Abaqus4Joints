function [R, dRdU, dRdf] = RESFUN(Uf, K, Fv, Lc, knl)
    fnl_pred = knl*Lc*Uf(1:end-1);  % Predicted normal force
    fnl = max(fnl_pred, 0);

    jnl = knl*Lc;
    jnl(fnl==0,:) = 0;

    R = K*Uf(1:end-1)+Lc'*fnl - Fv*Uf(end);
    dRdU = K+Lc'*Lc*knl;
    dRdf = -Fv;
end
