clear all
close all
%clc

%% SET UP
st = [0;1.9]; % starting position offset (applied to agents and cargo)
% map
map = png2BOMap('map_test_1.png',22);

% Cargo
center = [map.XWorldLimits(2) / 2 ; 2 - 1]; % [m]
center_mass = [0;0];        % [m]
dimensions = [1.5; 1];      % [m]
orientation = pi/2;         % [rad]

% Agents
param.range = 2;          % [m] max observable range
param.comm_range = 5;     % [m] max connection distance
param.radius = 0.1;         % [m] hitbox of the agent
param.N_rho = 90;           % division of the radius for discretization
param.N_phi = 90;           % division of the angle for discretization

cargo = rect_load(st + center, center_mass, orientation, dimensions);

%%
agents(1) = agent('James', [map.XWorldLimits(2) / 2 + 0.5; 2.5 - 1] + st, param, cargo, map);
agents(2) = agent('Pluto', [map.XWorldLimits(2) / 2 + 0.5; 1.5 - 1] + st, param, cargo, map);
agents(3) = agent('Gerlad',[map.XWorldLimits(2) / 2 - 0.5; 1.5 - 1] + st, param, cargo, map);
agents(4) = agent('Leila', [map.XWorldLimits(2) / 2 - 0.5; 2.5 - 1] + st, param, cargo, map);
agents(5) = agent('Samuel',[map.XWorldLimits(2) / 2 - 0.5; 2 - 1]   + st, param, cargo, map);

Ts = 10e-2;
sim_time = 2;

%% init the flock of robots
formation_limit = 0.05;

robots = flock(agents, cargo, Ts, 0);

robots.meetNeighbours();
robots.computeVisibilitySets();
robots.computeVoronoiTessellation();
robots.applyConstantDensity();
figure() % plot initial positions
subplot(1,3,1)
grid on
hold on
axis equal
show(map)
robots.plotVoronoiTessellationDetailed(3);
robots.plot()
hold off

%% Spread under the cargo
robots.spreadUnderCargo(10, 0.2, 4);
robots.attachAll();

subplot(1,3,2) % plot positions under cargo
show(map)
grid on
hold on
axis equal
robots.plotVoronoiTessellationDetailed(3);
robots.plot()
robots.plotCentroids();
hold off

%% Trajectory planning and movement 
k_WP = 0.01; % way point density factor
k_d = 10;  % movement gain toward the centroid
max_iterations = 30;

v = [0.1, 0.1, 0.1]';
w = [0.0, 0.2, 0.0]';
u = [v, w]; % move 30 centimeters upwards 
dest = robots.setTrajectory(u(1,:));
robots.setWayPoints(dest); % set destionation for each robot
            
robots.fixFormation();

robots.moveFormation(formation_limit, k_d, k_WP, max_iterations);

subplot(1,3,3) % plot positions after fixed formation movement
grid on
hold on
axis equal
show(map)
robots.plotVoronoiTessellationDetailed(1);
robots.plot();
robots.plotCentroids();
hold off

%% TESTING SECTION
% comment if not needed
%fprintf('TESTING SECTION\n');
% for simulation need a high resolution

%Testing_unit = tester(map, Ts, 1e-2);
%Testing_unit.runAll(true);