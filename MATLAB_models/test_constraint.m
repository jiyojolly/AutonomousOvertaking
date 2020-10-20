clf;
hold on;
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
inflation_factor = 1.005;
data.PredictionHorizon = 10;
obstcl = out.obstcl.Data(:,:,4)
obstcl = obstcl.*inflation_factor
ellip_coeff = fit_ellipse(obstcl(:,1),obstcl(:,2));

n=8;
a = ellip_coeff.a ; b = ellip_coeff.b; xe = ellip_coeff.X0_in; ye = ellip_coeff.Y0_in; phi = ellip_coeff.phi;
ellip = @(x,y) ((((x-xe).*cos(phi) - (y-ye).*sin(phi)))./a).^n + ((((x-xe).*sin(phi) + (y-ye).*cos(phi)))./b).^n - 1;
fimplicit(ellip, [-200 -180 60 70])

% ans = constraint_obstcl_avoid_alt(x,u,e,data,ellip_coeff n)