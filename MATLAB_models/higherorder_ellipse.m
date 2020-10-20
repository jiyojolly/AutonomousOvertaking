clf;
hold on;
n= 2;L = 5; W = 2; f =  nthroot(2,n); alpha = deg2rad(30);
rec = [ -L/2 -W/2; 
        -L/2  W/2;
        L/2   W/2;
        L/2+5 0;
        L/2  -W/2;
        -L/2 -W/2;]
R = [cos(alpha) -sin(alpha);
     sin(alpha) cos(alpha);];

rec = rec*R
plot(rec(:,1),rec(:,2))

% Ellipse parameters
% a = f*L/2 ; b = f*W/2; xe = 0; ye = 0; phi = alpha;
ellip_coeff_struct = fit_ellipse(rec(1:end-1,1),rec(1:end-1,2));
a = ellip_coeff_struct.a ; b = ellip_coeff_struct.b; xe = ellip_coeff_struct.X0_in; ye = ellip_coeff_struct.Y0_in; phi = ellip_coeff_struct.phi;


ellip = @(x,y) ((((x-xe).*cos(phi) - (y-ye).*sin(phi)))./a).^n + ((((x-xe).*sin(phi) + (y-ye).*cos(phi)))./b).^n - 1;
fimplicit(ellip, [-10 10 -10 10])