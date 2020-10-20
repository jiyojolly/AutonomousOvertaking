function ellip_coeff = getEllipseCoeff(obstcl)
    
    ellip_coeff = zeros(6,1);
    if nnz(obstcl) ~= 0
       ellip_coeff(:) = real(EllipseDirectFit(obstcl));
%     else
%         ellip_coeff = 
    end
end
