clear all
clc

%% SET UP

% map
map = png2BOMap('map_test_1.png',15);
% load
center = [1;1];             % [m]
center_mass = [0;0];        % [m]
dimensions = [1.5; 1];      % [m]
orientation = pi/2;         % [rad]

% Agents
param.range = 0.5;          % [m] max observable range
param.comm_range = 0.8;     % [m] max connection distance
param.radius = 0.1;         % [m] hitbox of the agent
param.N_rho = 60;           % division of the radius for discretization
param.N_phi = 60;           % division of the angle for discretization

cargo = rect_load(center, center_mass, orientation, dimensions);
agents(1) = agent('James',[1.5;1.5], param, cargo, map);
agents(2) = agent('Pluto',[0.5;0.5], param, cargo, map);
agents(3) = agent('Gerlad',[1.5;0.5],param, cargo, map);
agents(4) = agent('Leila',[0.5;1.5], param, cargo, map);
agents(5) = agent('Samuel',[1;1], param, cargo, map);

Ts = 10e-2;
sim_time = 2;

% init the flock of robots
my_robot_army = flock(agents, cargo, Ts);

% check neighbour
my_robot_army.meetNeighbours();
my_robot_army.computeVoronoiTessellation();
my_robot_army.computeVoronoiCentroids();


%% starting position plot

figure()
grid on
hold on
axis equal
show(map)
my_robot_army.plot()
my_robot_army.plotVoronoiTessellation();
my_robot_army.plotCentroids();
hold off
cmds = [0.3,0;  0.1,-pi/10; 0.1,-pi/10;  0.1,-pi/10]; 

path = my_robot_army.setTrajectory(cmds);

%% setting up operation

my_robot_army.spreadUnderCargo(round(sim_time / Ts));
my_robot_army.attachAll();

if(my_robot_army.checkBalance() == false)
    error("out of balance!!!")
end

%% ending position position
figure()
grid on
hold on
axis equal
show(map)
my_robot_army.plot()
my_robot_army.plotVoronoiTessellation();
my_robot_army.plotCentroids();
plotPath(path);
hold off
