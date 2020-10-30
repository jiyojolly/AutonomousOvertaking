clc;
clf;
hold on;
load('sim_out_sample.mat')
x = [-198.0000  , 55.0002,    1.5708  ,       0;
 -198.0006,   55.3936,    1.5713,    0.2000;
 -198.0024,   55.3993,    1.5727,    0.4000;
 -198.0053,   55.4089,    1.5750,    0.6000;
 -198.0094,   55.4223,    1.5782,    0.8000;
 -198.0135,   55.4399,    1.5813,    1.0000;
 -198.0187,   55.4614,    1.5852,    1.2000;
 -198.0249,   55.4868,    1.5897,    1.4000;
 -198.0323,   55.5160,    1.5951,    1.6000;
 -198.0409,   55.5491,    1.6011,    1.8000;
 -198.0509,   55.5860,    1.6081,    2.0000;];

u = [];
e = 0;
inflation_factor =1.0;
data.PredictionHorizon = 10;
obstcl = [out.obstcl.Data(1:2,:,12); out.obstcl.Data(4:end,:,12)];
pgon = polyshape(obstcl(:,1),obstcl(:,2));
plot(pgon);
% ellip_coeff = fit_ellipse(obstcl(:,1),obstcl(:,2));
[plgn_cntr(1) , plgn_cntr(2)] = pgon.centroid;

n=8;f=nthroot(2,n);
a = f*0.5*abs((abs(pgon.Vertices(1,1))-abs(pgon.Vertices(2,1)))*inflation_factor); b = f*0.5*abs((abs(pgon.Vertices(1,2))-abs(pgon.Vertices(3,2)))*inflation_factor); 
phi = deg2rad(-0);xe = plgn_cntr(1); ye = plgn_cntr(2);

ellip = @(x1,x2) ((((x1-xe).*cos(phi) - (x2-ye).*sin(phi)))./a).^n + ((((x1-xe).*sin(phi) + (x2-ye).*cos(phi)))./b).^n - 1; 
% fimplicit(ellip, [-210 -180 60 70])


% n=8;f=nthroot(2,n);
% a = f*0.5*abs((abs(pgon.Vertices(1,1))-abs(pgon.Vertices(2,1)))*inflation_factor); b = f*0.5*abs((abs(pgon.Vertices(1,2))-abs(pgon.Vertices(3,2)))*inflation_factor); 
% xe = plgn_cntr(1)+5; ye = plgn_cntr(2); phi = deg2rad(-0);
% ellip2 = @(x1,x2) ((((x1-xe).*cos(phi) - (x2-ye).*sin(phi)))./a).^n + ((((x1-xe).*sin(phi) + (x2-ye).*cos(phi)))./b).^n - 1;
% % fimplicit(ellip2, [-210 -180 60 70])


syms x1 x2 real
x = [x1;x2];
f = ellip(x1,x2)
fsurf(f,'ShowContours','on')
view(127,38)
gradf = jacobian(f,x)
hessf = jacobian(gradf,x)
% 
% fh = matlabFunction(f,gradf,hessf,'vars',{x});
% options = optimoptions('fminunc', ...
%     'SpecifyObjectiveGradient', true, ...
%     'HessianFcn', 'objective', ...
%     'Algorithm','trust-region', ...
%     'Display','final');
% [xfinal,fval,exitflag,output] = fminunc(fh,[-1;2],options)
% % ans = constraint_obstcl_avoid_alt(x,u,e,data,ellip_coeff n)