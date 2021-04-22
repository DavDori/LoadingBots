clear all
close all
clc

%% SET UP
st = [0;1.9]; % starting position offset (applied to agents and cargo)
% map
map = png2BOMap('map_test_1.png',22);

% Cargo
center = [map.XWorldLimits(2) / 2 ; 2]; % [m]
center_mass = [0;0];        % [m]
dimensions = [1.5; 1];      % [m]
orientation = pi/2;         % [rad]

% Agents
param.range = 2;          % [m] max observable range
param.comm_range = 5;     % [m] max connection distance
param.radius = 0.1;         % [m] hitbox of the agent
param.N_rho = 60;           % division of the radius for discretization
param.N_phi = 90;           % division of the angle for discretization

% cargo = rect_load(st + center, center_mass, orientation, dimensions);
% agents(1) = agent('James', [map.XWorldLimits(2) / 2 + 0.5; 2.5] + st, param, cargo, map);
% agents(2) = agent('Pluto', [map.XWorldLimits(2) / 2 + 0.5; 1.5] + st, param, cargo, map);
% agents(3) = agent('Gerlad',[map.XWorldLimits(2) / 2 - 0.5; 1.5] + st, param, cargo, map);
% agents(4) = agent('Leila', [map.XWorldLimits(2) / 2 - 0.5; 2.5] + st, param, cargo, map);
% agents(5) = agent('Samuel',[map.XWorldLimits(2) / 2 - 0.5; 2] + st, param, cargo, map);

Ts = 10e-2;
sim_time = 2;

% init the flock of robots
%my_robot_army = flock(agents, cargo, Ts);



%% TESTING SECTION
% comment if not needed
fprintf('TESTING SECTION\n');
% for simulation need a high resolution
param.N_phi = 50;
param.N_rho = 30;
Testing_unit = tester(map, param, Ts, 1e-2);
Testing_unit.fixedFormation(true);
%Testing_unit.collisionDetection(true);
%Testing_unit.runAll(true);