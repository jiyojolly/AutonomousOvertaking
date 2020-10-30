function [G,Gmv,Ge] = constraint_obstcl_avoid_test_jacobian(X,U,e,data, ellip_coeff)

    p = data.PredictionHorizon;
    Nx = data.NumOfStates;
    Nc = 2*p;
    G = zeros(p,Nx,Nc);
    
    X1 = X(2:p+1,1);
    X2 = X(2:p+1,2);

    
    G(1:p,2,1:p) = diag(2*X1 - 3);
    
    Nmv = length(data.MVIndex);
    Gmv = zeros(p,Nmv,Nc);
    Gmv(1:p,1,p+1:2*p) = diag(2*U(1:p,data.MVIndex(1)));
    
     Ge = zeros(Nc,1);