function ellip_coeff = getEllipseCoeff_alt(obstcl, obstcl_ellip_order, inflation_factor)
    
    ellip_coeff = zeros(6,1);
    if nnz(obstcl) ~= 0
        ellip_coeff_struct = fit_ellipse(obstcl(:,1),obstcl(:,2));
        ellip_coeff(1:5) = [ellip_coeff_struct.a*inflation_factor ellip_coeff_struct.b*inflation_factor ellip_coeff_struct.X0_in ellip_coeff_struct.Y0_in ellip_coeff_struct.phi ];
        ellip_coeff(6) = obstcl_ellip_order;
%   else
%       ellip_coeff = 
    end
end
