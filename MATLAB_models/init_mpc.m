clear;
nx = 4; ny = 4; nu = 2;
Ts = 0.02;
PredHor = 10;
CntrlHor = 5;

ellip_coeff = [0 0 0 0 0 0];

x = [0 1 2 3];
mv = [1 1];

mpc_planner = nlmpc(nx,ny,nu);
mpc_planner.Ts = Ts; 
mpc_planner.PredictionHorizon = PredHor;
mpc_planner.ControlHorizon = CntrlHor;
mpc_planner.Model.StateFcn = "vkinematicmodel_bicycle";
mpc_planner.Model.IsContinuousTime = false;
mpc_planner.Model.NumberOfParameters = 1;
mpc_planner.Optimization.CustomIneqConFcn = "constraint_obstcl_avoid";
% mpc_planner.Jacobian.CustomIneqConFcn = "constraint_obstcl_avoid_jacobian";


createParameterBus(mpc_planner,['MPC_Planner_Controller/MPC/Nonlinear MPC Controller'],'params',{ellip_coeff});
x0 = [0.1;0.2;-pi/2;0.3];
u0 = [0.4; 0.0];
validateFcns(mpc_planner, x0, u0, [], {ellip_coeff});