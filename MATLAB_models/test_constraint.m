x = [1,2;
    2,3;
    -2,1;
    0,0];
u = [];
data.PredictionHorizon = 3;
obstcl = [-2 1;
           2 1;
         3.35 0;
           2 -1;
           -2 -1];

constraint_obstcl_avoid(x,u,data,obstcl)