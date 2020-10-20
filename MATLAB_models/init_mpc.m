clear;
nx = 4; ny = 4; nu = 2;
T_horizon = 1;
Ts = 0.05;
PredHor = T_horizon/Ts
CntrlHor = PredHor/2
enable_MPC = 0;

obstcl_ellip_order = 2;
ellip_coeff = [0 0 0 0 0 obstcl_ellip_order];
inflation_factor = 1.007;

x = [0 1 2 3];
mv = [1 1];

mpc_planner = nlmpc(nx,ny,nu);
mpc_planner.Ts = Ts; 
mpc_planner.PredictionHorizon = PredHor;
mpc_planner.ControlHorizon = CntrlHor;
mpc_planner.Model.StateFcn = "vkinematicmodel_bicycle";
mpc_planner.Model.IsContinuousTime = true;
mpc_planner.Model.NumberOfParameters = 1;
mpc_planner.Optimization.CustomIneqConFcn = "constraint_obstcl_avoid_alt";
% mpc_planner.Jacobian.CustomIneqConFcn = "constraint_obstcl_avoid_jacobian";

%Apply limits on input
mpc_planner.ManipulatedVariables(1).Min = -10;
mpc_planner.ManipulatedVariables(2).Min = deg2rad(-70);
mpc_planner.ManipulatedVariables(1).Max = 10;
mpc_planner.ManipulatedVariables(2).Max = deg2rad(70);


%Weights
mpc_planner.Weights.OutputVariables = [15 15 0 0];

createParameterBus(mpc_planner,['MPC_Planner_Controller/MPC/Nonlinear MPC Controller'],'params',{ellip_coeff});
x0 = [0.1;0.2;-pi/2;0.3];
u0 = [0.4; 0.0];
validateFcns(mpc_planner, x0, u0, [], {ellip_coeff});