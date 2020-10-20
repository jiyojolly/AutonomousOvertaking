function cineq = constraint_obstcl_avoid_alt(x,u,e,data, ellip_coeff)
    
    if nnz(ellip_coeff(1:5)) ~= 0
        %Ellipse parameters
        n=ellip_coeff(6);
        a = ellip_coeff(1) ; b = ellip_coeff(2); xe = ellip_coeff(3); ye = ellip_coeff(4); phi = ellip_coeff(5);
        ellip = @(x,y) ((((x-xe).*cos(phi) - (y-ye).*sin(phi)))./a).^n + ((((x-xe).*sin(phi) + (y-ye).*cos(phi)))./b).^n - 1;
               
        p = data.PredictionHorizon;
    
        x1 = x(2:p+1,1);
        y1 = x(2:p+1,2);

        cineq = [-(ellip(x1,y1));];
    else 
        cineq = 0;
    end
end