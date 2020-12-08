function cineq = constraint_obstcl_avoid(X,U,e,data, ellip_coeff)

    p = data.PredictionHorizon;
    U1 = U(1:p,data.MVIndex(1));
    X1 = X(2:p+1,1);
    X2 = X(2:p+1,2);

    cineq = [2*X1.^2 - 3*X2 - 10;
             U1.^2 - 5];
end
