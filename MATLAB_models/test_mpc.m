clear;
tr
rosinit;
nx = 4;
ny = 4;
nu = 2;
Tf = 4;
Ts = 0.02;
T = 0:Ts:Tf;
x = [0 1 2 3];
mv = [1 1];
nlobj = nlmpc(nx,ny,nu);

ego = struct;
ego.x = x;
ego.Length = 5;
ego.Width = 2;

%% 
% The obstacle in this example is a nonmoving object in the middle of the
% center lane with the same size as the ego car.
obstacle = struct;
obstacle.Length = 5;
obstacle.Width = 2;

%% 
% Place the obstacle |50| meters down the road.
obstacle.X = 10;
obstacle.Y = 0;
obstacle.v = 2.0;

% Plot the Ego vehicle.
p1 = plot(ego.x(1),ego.x(2),'gx'); hold on; grid on;
% rectangle('Position',[ego.x(1)-ego.Length/2,ego.x(2)-ego.Width/2,ego.Length,ego.Width]);

p1.XDataSource = 'ego.x(1)';
p1.YDataSource = 'ego.x(2)';

% % Plot the static obstacle.
p2 = plot(obstacle.X,obstacle.Y,'rx');
% % rectangle('Position',[obstacle.X-obstacle.Length/2,obstacle.Y - obstacle.Width/2,obstacle.Length,obstacle.Width]);

p2.XDataSource = 'obstacle.X';
p2.YDataSource = 'obstacle.Y';

% Reset the axis.
axis([0 50 -20 20]);
xlabel('X');
ylabel('Y');
title('Obstacle Avoidance Maneuver');

vehicle_status_sub = rossubscriber('/carla/ego_vehicle/vehicle_status');

vehicle_status = receive(vehicle_status_sub,10);
print(vehicle_status)

for k = 1:length(T)
    ego.x(1) = ego.x(1) + (ego.x(4)*Ts);
    obstacle.X = obstacle.X + (obstacle.v*Ts);
    refreshdata
    drawnow    
end

% nlobj.Model.StateFcn = "vkinematicmodel_bicycle"
% validateFcns(nlobj,x,mv)


