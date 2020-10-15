function cineq = constraint_obstcl_avoid(x,u,e,data, obstcl)

    xy = obstcl;
    ellip_coeff = EllipseDirectFit(xy);
    ellip = @(x,y) ellip_coeff(1)*x.^2 + ellip_coeff(2)*x.*y +...
                   ellip_coeff(3)*y.^2 + ellip_coeff(4)*x +...
                   ellip_coeff(5)*y + ellip_coeff(6);
               
%     fimplicit(ellip)
    p = data.PredictionHorizon;
    x1 = x(2:p+1,1);
    y1 = x(2:p+1,2);

    cineq = [ellip(x1,y1);];