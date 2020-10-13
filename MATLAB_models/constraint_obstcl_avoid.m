function cineq = constraint_obstcl_avoid(x,u,data, ac,bc,c)

    p = data.PredictionHorizon;
    X1 = X(2:p+1,1);
    X2 = X(2:p+1,2);

    cineq = [2*X1.^2 - 3*X2 - 10;];