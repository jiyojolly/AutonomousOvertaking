function plotAll(x_curr, x_ref_curr, obstcl, x_pred, mv_pred)

%     coder.extrinsic('fcn'); 
%     coder.extrinsic('fimplicit');
    clf;
    hold on;
    x_curr
    x_ref_curr
    obstcl
    x_pred
    mv_pred
    if nnz(obstcl) ~= 0

        %Plot Ego car
        scatter(x_curr(1),x_curr(2),'d')
        
        %PLot MPC Referece  
        scatter(x_ref_curr(1),x_ref_curr(2),'MarkerFaceColor',[0 .7 .7])
        
        %Plot MPC predicted path      
        scatter(x_pred(:,1),x_pred(:,2),'filled')
        
        %Plot obstacle
        pgon = polyshape(obstcl(:,1),obstcl(:,2));
        plot(pgon);


        %Plot fitted ellipse
        ellip_coeff = zeros(6,1);
        ellip_coeff(:) = real(EllipseDirectFit(obstcl));   
        ellip = @(x,y) ellip_coeff(1)*x.^2 + ellip_coeff(2)*x.*y +...
                       ellip_coeff(3)*y.^2 + ellip_coeff(4)*x +...
                       ellip_coeff(5)*y + ellip_coeff(6);

        fimplicit(ellip, [-200 -180 60 70])
    end
    
end