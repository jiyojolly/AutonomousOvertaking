function plotAll(x, obstcl, x_pred)

%     coder.extrinsic('fcn'); 
%     coder.extrinsic('fimplicit');
    clf;
    hold on;
    obstcl
    x
    x_pred
    if nnz(obstcl) ~= 0

        %Plot Ego car
        scatter(x(1),x(2))
        %Plot MPC predicted path 
        sz = 25;
        c = linspace(1,10,length(x_pred(1,:)));
        scatter(x_pred(1,:),x_pred(2,:),sz,c,'filled')
        xy = obstcl;
        %Plot obstacle
        pgon = polyshape(xy(:,1),xy(:,2));
        plot(pgon);
% 
% 
%         %Plot fitted ellipse
%         ellip_coeff = zeros(6,1);
%         ellip_coeff(:) = real(EllipseDirectFit(xy));   
%         ellip = @(x,y) ellip_coeff(1)*x.^2 + ellip_coeff(2)*x.*y +...
%                        ellip_coeff(3)*y.^2 + ellip_coeff(4)*x +...
%                        ellip_coeff(5)*y + ellip_coeff(6);
% 
%         fimplicit(ellip, [-200 -180 60 70])
    end
    
end