%Plot fitted ellipse
% ellip_coeff = [1.00390613079071;2.07959103584290;-198.002883911133;55.0002021789551;deg2rad(90-(90));6]
% n=ellip_coeff(6);
% a = ellip_coeff(1) ; b = ellip_coeff(2); xe = ellip_coeff(3); ye = ellip_coeff(4); phi = ellip_coeff(5);
% ellip = @(x,y) ((((x-xe).*cos(phi) - (y-ye).*sin(phi)))./a).^n + ((((x-xe).*sin(phi) + (y-ye).*cos(phi)))./b).^n - 1;
% 
% fimplicit(ellip, [-210 -190 50 70])

polyin = polyshape([0 0 1 1],[2 0 0 2]);
poly1 = rotate(polyin,45);
plot(poly1)