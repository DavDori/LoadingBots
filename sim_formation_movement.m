%% DODGE SIMULATION WITH MORE AGENTS AND CONSTRAINED MOVEMENT UNDER CARGO
clear all
close all
clc

%% SET UP

st = [0;-1]; % starting position offset (applied to agents and cargo)
% map
map_res = 50; % map_test_1 : 22
map = png2BOMap('map_test_3_high_res.png', map_res);
map_width =  map.XLocalLimits(2) - map.XLocalLimits(1);
map_height = map.YLocalLimits(2) - map.YLocalLimits(1);

% Cargo parameters
center = [map.XWorldLimits(2) / 2 ; map.YWorldLimits(2) / 2]; % [m]
cargo_p.center_mass = [0;0];        % [m]
cargo_p.dimensions =  [1.5; 1.5];   % [m]
cargo_p.orientation = pi / 2;       % [rad]

% Agents
n_agents = 5;                % number of agents
division_starting_spots = 3; % define the starting areas
agent_p.range = 1.5;         % [m] max observable range
agent_p.comm_range = 1.5;    % [m] max connection distance
agent_p.radius = 0.05;       % [m] hitbox of the agent
agent_p.N_rho = 100;          % division of the radius for discretization
agent_p.N_phi = 60;          % division of the angle for discretization
agent_p.max_speed = 0.5;     % [m/s] maximum speed reachable by the agent


% Simulation parameters----------------------------------------------------
movie = true;
steps_per_frame = 1; % take 1 frame every x steps of simulation

Ts = 1e-2;         % sampling time
sim_time = 16;     % total simulation time
slow_factor = 1;   % x1 speed of visualization

% Centroids gains ---------------------------------------------------------

kp_formation = 5;%6;   % formation centroid gain

SUC_steps = 30;     % spread under cargo steps
offset_cargo = 0.1; % [m] offset from cargo shape where robots can go in the spread under cargo phase

bound = 0.10; % buonds to keep when in formation
wp_density_spread = 0.03; % waypoint density spread for formation movement

% cargo trajectory
fw = [0.08, 0];
sl = [0.08, deg2rad(1)];  % solf left
sr = [0.08, -deg2rad(1)]; % soft right
ml = [0.05, deg2rad(3)];  % medium left
mr = [0.05, -deg2rad(3)]; % medium right
u = [sl; sl; sl; fw; fw; sr; sr; sr; fw; fw];
 
wp_error = 0.02;
max_iterations = 200;
%% objects initialization

cargo = RectangularCargo(st + center, cargo_p.center_mass, cargo_p.orientation,...
    cargo_p.dimensions);


% set initial positions as random uniformly distributed under the cargo,
% considering the biggest square that is still under the cargo itself

names = {'Bob', 'Jet', 'Zoe', 'Tim', 'Lue', 'Hari', '007', 'Gaal', 'Tif',...
    'Hug', 'Mug', 'May','Emy', 'Jim', 'Pos', 'NaN', 'Sin', 'Cos', 'Mem', 'PLC'};

pos_relative = randomStartingPositions(cargo_p.dimensions, division_starting_spots...
          , n_agents) - ones(2,1) .* cargo_p.dimensions / 2;
    
for i = 1:n_agents
    pos = cargo.relative2absolute(pos_relative(:,i));
    agents(i) = agent(names(i), pos, agent_p, cargo, map);
end

robots = flock(agents, cargo, Ts, 0);

%% Represent cargo trajectory


way_points = robots.setTrajectory(u);



%% Video setup

steps = fix(sim_time / Ts);
if(movie == true)
    h = figure();
    h.Visible = 'off';
    axis tight manual
    ax = gca;
    ax.NextPlot = 'replaceChildren';
    v = VideoWriter('sim_formation.avi');
    v.FrameRate = fix(slow_factor * steps / (sim_time * steps_per_frame));
    open(v);
end

%% Spread under cargo (first phase)

for i = 1:SUC_steps
    bar = loadingBar(i, SUC_steps, 20, '#');
    disp(bar);
    robots.meetNeighbours();
    robots.sendScan([]); 
    robots.computeVisibilitySets(); % no obstacle is present in this phase
    robots.liberalConnectivityMaintenance('All', 'All');
    robots.computeVoronoiTessellationCargo(offset_cargo);
    robots.applyFarFromCenterMassDensity(5);
    robots.applyConstantDensity('Obstacle');
    robots.computeVoronoiCentroids();
    robots.moveToCentroids(kp_formation);
    
    % save the frame
    if(rem(i, steps_per_frame) == 0 && movie == true)
        % video setup
        hold on         
        xlim([0,map_width]);
        ylim([0,map_height]);
        axis equal
        grid on
        show(map);
        %robots.plotVoronoiTessellationDetailed(5, 20);
        robots.plot(false);
        %robots.plotCentroids('Formation');
        hold off

        frm = getframe(gcf);
        clf(h);
        writeVideo(v, frm);
    end
end
disp('Spread under cargo phase concluded...');

%% algorithm


% attach all agents 
robots.attach();
% save positions for path planning
robots.saveCargoPosition();
%simulation hystory data
positions_history_x = zeros(steps, robots.n_agents);
positions_history_y = zeros(steps, robots.n_agents);

i = 1;
j = 1; % initialize waypoint counter
k = 1;

robots.setWayPoints(way_points(j,:));
wps = robots.getCurrentWayPoints();
robots.fixFormation('Attached'); % set distances between agents to keep

while(j < size(way_points, 1)+1 && i < max_iterations)
    current_pose = robots.getPositions();
    diff = robots.getCurrentWayPoints() - current_pose;
    disp(sqrt(diff(:,1).^2 + diff(:,2).^2)');
    positions_history_x(k,:) = current_pose(:,1)';
    positions_history_y(k,:) = current_pose(:,2)';
    disp(strcat(string(j), ' out of : ', string(size(way_points, 1)), ...
        ' current iteration : ', string(i)));
    i = i + 1;
    k = k + 1;
    
    robots.meetNeighbours();
    robots.sendScan([]); 

    robots.computeVisibilitySets([]);
    robots.connectivityMaintenanceFF(bound);
    robots.computeVoronoiTessellationFF(bound);

    % all robots should tend to return to the original position under cargo
    robots.applyWayPointDensity(wp_density_spread); 

    robots.computeVoronoiCentroids();
    
    % next step of simulation
    % follows the vectorial sum of both vectors defined by the centroids.
    % In some cases the obstacle should be already considered by the
    % formation cell but for non constant densities it has a different
    % weight
    robots.moveToCentroids(kp_formation, 0, 'All');
    reached_way_point = robots.areWayPointsReached(wp_error);
    if(all(reached_way_point) == true && j < size(way_points, 1)) % all the waypoints have been reached
        j = j + 1; % set index for next way-point
        i = 1; % reset iterations
        robots.setWayPoints(way_points(j,:));
        disp('way point reached')
    end
    
    if(rem(i, steps_per_frame) == 0 && movie == true)
        % video setup
        hold on         
        xlim([0,map_width]);
        ylim([0,map_height]);
        show(map);
        %robots.plotVoronoiTessellationDetailed(1, 40);
        robots.plot(false);
        hold off

        frm = getframe(gcf);
        clf(h);
        writeVideo(v, frm);
    end
end

if(movie == true)
    h.Visible = 'on';
    close(v);
end

%% plots 
figure()
grid on
hold on
title('Trajectory hystory')
set(gcf,'color','w');
show(map)
for i = 1:robots.n_agents
    plot(positions_history_x(1:k-1,i), positions_history_y(1:k-1,i),'DisplayName',...
        string(robots.agents(i).name));
    plot(positions_history_x(k-1,i), positions_history_y(k-1,i), 'xr');
    plot(positions_history_x(1,i), positions_history_y(1,i), 'og');
end

wp = robots.getCurrentWayPoints();
plot(way_points(:,1), way_points(:,2), 'b', 'DisplayName', 'input trajectory', 'LineWidth', 2);
legend
    