clear all
clc

%% SET UP

% load
center = [1;1];             % [m]
center_mass = [0.1;0.1];    % [m]
dimensions = [1.5; 1];      % [m]
orientation = pi/4;         % [rad]

% Agents
param.range = 1;            % [m] max observable range
param.comm_range = 0.8;     % [m] max connection distance
param.radius = 0.1;         % [m] hitbox of the agent
param.N_rho = 90;           % division of the radius for discretization
param.N_phi = 90;           % division of the angle for discretization

loadObj = rect_load(center, center_mass, orientation, dimensions);
agents(1) = agent('James',[1.2;1.3], param, loadObj);
agents(2) = agent('Pluto',[0.6;0.7], param, loadObj);
agents(3) = agent('Gerlad',[1.2;0.7],param, loadObj);
agents(4) = agent('Leila',[0.7;1.1], param, loadObj);

agents(1).attach();
agents(2).attach();
agents(3).attach();
agents(4).attach();

Ts = 10e-2;

my_robot_army = flock(agents, loadObj, Ts);

% check neighbour
my_robot_army.meetNeighbours();
my_robot_army.computeVoronoiTessellation();
centroids = my_robot_army.computeVoronoiCentroids();


figure()
grid on
hold on
my_robot_army.plot()
my_robot_army.plotVoronoiTessellation();
my_robot_army.plotCentroids();
hold off