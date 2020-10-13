function completed = bicycle_model_test()
close all;
% bicycle_model_test - example of 
%     nonlinear reachability analysis for following a reference trajectory; 
%     this example is also a unit test function.
%
%     This example is similar to the one in [1].q
%
%     One of the differences is that computational accelerations such as 
%     parallelization are not activated for this example.
%     Further accelerations, such as taking advantage of monotonicity
%     in the Lagrange remainder are also not considered.
%
% Syntax:  
%    completed = bicycle_model_test()
%
% Inputs:
%    no
%
% Outputs:
%    completed - boolean 
%
% References:
%    [1] M. Althoff and J. M. Dolan. Online verification of automated
%        road vehicles using reachability analysis.
%        IEEE Transactions on Robotics, 30(4):903-918, 2014.

% Author:       Jiyo Palatti
% Written:      02-September-2020
% Last update:  02-September-2020 (restucture params/options)
% Last revision:---

%------------- BEGIN CODE --------------

% Parameters --------------------------------------------------------------

params.tFinal = 1.0;

% intial set
params.R0 = zonotope([[0.0; 0.0; 0.0; 0.0],...
                      0.05*diag([1, 1, 1, 1])]);


% uncertain inputs
params.U = zonotope(interval([0.0; -pi/3], ...
                               [5.0; pi/3]));


% Reachability Settings ---------------------------------------------------

options.timeStep = 0.01;        % Time step size
options.taylorTerms = 5;        % Taylor terms
options.zonotopeOrder = 30;     % Zonotope order

options.alg                  = 'lin';
options.tensorOrder          = 3;
options.errorOrder           = 5;
options.intermediateOrder    = 10;
% options.maxError             = Inf(dim(params.R0),1);
% options.reductionInterval    = Inf;

% System Dynamics ---------------------------------------------------------

vehicle = nonlinearSys('bicycle_model',@vkinematicmodel_bicycle,4,2);


% Reachability Analysis ---------------------------------------------------

tic
R = reach(vehicle, params, options);
tComp = toc;
disp(['computation time of reachable set: ',num2str(tComp)]);


% Simulation --------------------------------------------------------------

% simulation settings
simOpt.points = 60;
simOpt.fracVert = 0.5;
simOpt.fracInpVert = 0.5;
simOpt.inpChanges = 6;

% random simulation
simRes = simulateRandom(vehicle, params, simOpt);


% Visualization -----------------------------------------------------------

dims = {[1 2],[3 4]};


for k = 1:length(dims)
    
    figure; hold on; box on
    projDims = dims{k};

    % plot reachable sets 
    plot(R,projDims,'FaceColor',[.8 .8 .8],'EdgeColor','none','Order',3);
    
    % plot initial set
    plot(params.R0,projDims,'w','Filled',true,'EdgeColor','k');
    
    % plot simulation results     
    plot(simRes,projDims);

    % label plot
    xlabel(['x_{',num2str(projDims(1)),'}']);
    ylabel(['x_{',num2str(projDims(2)),'}']);
end

% example completed
completed = 1;

%------------- END OF CODE --------------