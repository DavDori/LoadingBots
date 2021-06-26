%% DODGE SIMULATION WITH MORE AGENTS AND FREE MOVEMENT UNDER CARGO
clear all
close all
clc

%% SET UP
st = [0;0]; % starting position offset (applied to agents and cargo)
% map
map = png2BOMap('map_test_1.png',22);
map_width = map.XLocalLimits(2) - map.XLocalLimits(1);
map_height = map.YLocalLimits(2) - map.YLocalLimits(1);

% Cargo
center = [map.XWorldLimits(2) / 2 ; 2 - 1]; % [m]
center_mass = [0;0];        % [m]
dimensions = [1.5; 1];      % [m]
orientation = pi/2;         % [rad]

% Agents
param.range = 2;          % [m] max observable range
param.comm_range = 5;     % [m] max connection distance
param.radius = 0.1;         % [m] hitbox of the agent
param.N_rho = 36;           % division of the radius for discretization
param.N_phi = 36;           % division of the angle for discretization

formation_limit = 0.05;

% ball
ball_starting_point = [0.5; 0.5];
ball_r = 0.2; % [m]
ball_speed = 1;
ball_direction = deg2rad(10);

Ts = 1e-1;
sim_time = 2;

kp_formation = 1;
kp_obstacle = 2;
offset_cargo = 0.1;

bound = 0.05; % for fixed formation

% attaching 
param_at.kp = 1;
param_at.th = 0.1;

% detaching
Kp_d = 1;
th_d = 0.18;
%% objects initialization

cargo = rect_load(st + center, center_mass, orientation, dimensions);

agents(1) = agent('James', [map.XWorldLimits(2) / 2 + 0.5; 2.5 - 1] + st, param, cargo, map);
agents(2) = agent('Pluto', [map.XWorldLimits(2) / 2 + 0.5; 1.5 - 1] + st, param, cargo, map);
agents(3) = agent('Gerlad',[map.XWorldLimits(2) / 2 - 0.5; 1.5 - 1] + st, param, cargo, map);
agents(4) = agent('Leila', [map.XWorldLimits(2) / 2 - 0.5; 2.5 - 1] + st, param, cargo, map);
agents(5) = agent('Samuel',[map.XWorldLimits(2) / 2 - 0.5; 2 - 1]   + st, param, cargo, map);
agents(6) = agent('Anakin',[map.XWorldLimits(2) / 2 + 0.5; 2 - 1]   + st, param, cargo, map);

robots = flock(agents, cargo, Ts, 0);

ball_v = [cos(ball_direction); sin(ball_direction)] * ball_speed;
ball = Obstacle(ball_r, ball_starting_point, ball_v, Ts);
global test;

%% spread under the cargo
test = 41;
steps = 40;
robots.spreadUnderCargo(15, 0.2, 1);
robots.attach();
last_d = zeros(robots.n_agents, 1);

%% Starting situation
figure()
hold on         
xlim([0,map_width]);
ylim([0,map_height]);
axis equal
grid on
show(map);
%robots.plotVoronoiTessellationDetailed(1);
robots.plot();
robots.plotCentroids();
ball.plot();
hold off
    
%% P algorithm 
h = figure();
h.Visible = 'off';
axis tight manual
ax = gca;
ax.NextPlot = 'replaceChildren';
GIF(steps) = struct('cdata',[],'colormap',[]);
v = VideoWriter('sim_P.avi');
v.FrameRate = 10;
open(v);

% set positions to hold

hold_positions = robots.getAgentsPositions('All');


for i = 1:steps
    
    robots.meetNeighbours();
    robots.sendScan(ball); 
    
    robots.fixFormation('Attached');
            
    robots.computeVisibilitySets(ball);
    %robots.connectivityMaintenanceFF(bound, 'Attached');
    %robots.connectivityMaintenance('Detached');
    
    %robots.computeVoronoiTessellationFF(bound, 'Attached');
    robots.computeVoronoiTessellationCargo(offset_cargo);
    
    robots.applyConstantDensity('Obstacle');
    robots.applyMultiplePointsDensity(hold_positions, 0.2, 'All');
    
    id_a = robots.attachable('P', param_at);
    [id_d, val_d] = robots.priorityRankingP(Kp_d);
    if(isempty(id_d) == false) % there is at least a robot to deattach
        if(val_d > th_d) % agent wants to move
            robots.detach(id_d);
        end
    end
    
    % video setup
    hold on         
    xlim([0,map_width]);
    ylim([0,map_height]);
    axis equal
    grid on
    show(map);
    robots.plotVoronoiTessellationDetailed(1);
    robots.plot();
    robots.plotCentroids();
    ball.plot();
    hold off
%     
    GIF(i) = getframe(gcf);
    clf(h);
    writeVideo(v, GIF(i));
    robots.moveToCentroids(kp_formation, kp_obstacle, 'Detached');
    ball.move();
    robots.print();
end
h.Visible = 'on';
close(v);