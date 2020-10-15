clf;
% Car parameters
car_len = 4;
car_wid = 2;
car_loc = [0, 0];
vel_fac = 1.5;
vel_fac_lead = 0.9; 

x = [ car_loc(1)-car_len/2 car_loc(1)+car_len/2 car_loc(1)+car_len/2+vel_fac*vel_fac_lead car_loc(1)+car_len/2 car_loc(1)-car_len/2]
y = [                   car_loc(2)+car_wid/2 car_loc(2)+car_wid/2 car_loc(2)                   car_loc(2)-car_wid/2 car_loc(2)-car_wid/2]
pgon = polyshape(x,y);
plot(pgon)
hold on;

xy = [x;y]'
ellip_coeff = EllipseDirectFit(xy)



ellip = @(x,y) ellip_coeff(1)*x.^2 + ellip_coeff(2)*x.*y + ellip_coeff(3)*y.^2 + ellip_coeff(4)*x + ellip_coeff(5)*y + ellip_coeff(6)
test_points = [1,2;2,3;-2,1;0,0]
ellip(test_points(:,1), test_points(:,2)) 
fimplicit(ellip)



% x = [ -50 car_loc(1)-car_len/2-vel_fac -car_len/2 0         car_len/2 car_loc(1)+car_len/2+vel_fac*vel_fac_lead 50];
% y = [ -50 car_loc(1)                    car_wid/2 car_wid/2 car_wid/2 car_loc(1) -50 ];
% pts = [x ; y]
% pts_rot = rot2d(pi/2, pts)
% 
% 
% xx = [-50:0.1:50];
% spline = csape(pts(1,:),pts(2,:));
% spline_rot = csape(pts_rot(1,:),pts_rot(2,:));
% 
% polynom = polyfit(pts(1,:),pts(2,:), 4)
% 
% plot(xx, fnval(spline,xx),'k-',x,y,'ro');
% plot(xx, fnval(spline_rot,xx),'k-',x,y,'ro');
% % plot(xx, polyval(polynom,xx),'k-',x,y,'ro');
% scatter(pts_rot(1,:), pts_rot(2,:))
% % spline_dx = fnder(spline);
% % plot(xx, fnval(spline_dx,xx),'k-',x,y,'ro');
% % spline_dxx = fnder(spline,2);
% % plot(xx, fnval(spline_dxx,xx),'k-',x,y,'ro');
xlim([-20,20]);
ylim([-20,20]);
