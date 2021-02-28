clear all
close all
clc

%% SET UP

% map
map = png2BOMap('map_test_1.png',22);

% Cargo
center = [map.XWorldLimits(2) / 2 ; 2]; % [m]
center_mass = [0;0];        % [m]
dimensions = [1.5; 1];      % [m]
orientation = pi/2;         % [rad]

% Agents
param.range = 0.8;          % [m] max observable range
param.comm_range = 2.0;     % [m] max connection distance
param.radius = 0.1;         % [m] hitbox of the agent
param.N_rho = 60;           % division of the radius for discretization
param.N_phi = 60;           % division of the angle for discretization

cargo = rect_load(center, center_mass, orientation, dimensions);
agents(1) = agent('James', [map.XWorldLimits(2) / 2 + 0.5; 2.5], param, cargo, map);
agents(2) = agent('Pluto', [map.XWorldLimits(2) / 2 + 0.5; 1.5], param, cargo, map);
agents(3) = agent('Gerlad',[map.XWorldLimits(2) / 2 - 0.5; 1.5],param, cargo, map);
agents(4) = agent('Leila', [map.XWorldLimits(2) / 2 - 0.5; 2.5], param, cargo, map);
agents(5) = agent('Samuel',[map.XWorldLimits(2) / 2 - 0.5; 2], param, cargo, map);

Ts = 10e-2;
sim_time = 2;

% init the flock of robots
my_robot_army = flock(agents, cargo, Ts);
%my_robot_army.print();

my_robot_army.meetNeighbours(); 
my_robot_army.computeVoronoiTessellationCargo(0.10);
%my_robot_army.applyFarFromCenterMassDensity();
%my_robot_army.applySinglePointDensity(cargo.center, 0.01);
my_robot_army.computeVoronoiCentroids();

my_robot_army.print();

figure()
grid on
hold on
axis equal
show(map)
my_robot_army.plotVoronoiTessellation();
my_robot_army.plot()
my_robot_army.plotCentroids();

hold off

%% starting position plot

% figure()
% grid on
% hold on
% axis equal
% show(map)
% my_robot_army.plot();
% hold off
% 
% %% setting up operation
% 
% offset = 0.2; %[m] offset from the cargo perimeter
% 
% my_robot_army.spreadUnderCargo(round(sim_time / Ts), offset);
% my_robot_army.attachAll();
% 
% if(my_robot_army.checkBalance() == false)
%     error("out of balance!!!")
% end
% % save current positions of the agents as reference for the
% % formation shape
% my_robot_army.fixFormation();
% 
% %% ending position after spreading under the cargo
% figure()
% grid on
% hold on
% axis equal
% show(map)
% my_robot_army.plot()
% my_robot_army.plotVoronoiTessellation();
% my_robot_army.plotCentroids();
% hold off
% 
% %% movement towards a set position
% cmds = [0.3,0;  0.1,0; 0.1,0;  0.1,0]; 
% 
% path = my_robot_army.setTrajectory(cmds);
% spread_factor = 0.1;
% % initial status
% my_robot_army.meetNeighbours(); 
% my_robot_army.computeVoronoiTessellation();
% my_robot_army.applyIdealPointDensity(spread_factor);
% my_robot_army.computeVoronoiCentroids();
%     
% figure()
% grid on
% hold on
% axis equal
% show(map)
% my_robot_army.plotAgentsPath(path(1:2,:));
% my_robot_army.plot()
% my_robot_army.plotCentroids();
% my_robot_army.plotVoronoiTessellation();
% hold off

% my_robot_army.setWayPoints(path(end,:));
% for i = 1:20
%     my_robot_army.meetNeighbours(); 
%     my_robot_army.computeVoronoiTessellation();
%     my_robot_army.applyIdealPointDensity();
%     my_robot_army.computeVoronoiCentroids();
%     my_robot_army.moveToCentroids(2);
% end
% 
% figure()
% grid on
% hold on
% axis equal
% show(map)
% my_robot_army.plot()
% my_robot_army.plotCentroids();
% my_robot_army.plotVoronoiTessellation();
% hold off
