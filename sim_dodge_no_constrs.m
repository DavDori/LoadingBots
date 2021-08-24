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
center = [map.XWorldLimits(2) / 2 ; map.YWorldLimits(2) / 2]; % [m]
cargo_p.center_mass = [0;0];        % [m]
cargo_p.dimensions =  [1.5; 1.5];   % [m]
cargo_p.orientation = pi/2;         % [rad]

% Agents
n_agents = 12;               % number of agents
division_starting_spots = 4; % define the starting areas
agent_p.range = 2.5;         % [m] max observable range
agent_p.comm_range = 5;      % [m] max connection distance
agent_p.radius = 0.05;       % [m] hitbox of the agent
agent_p.N_rho = 36;          % division of the radius for discretization
agent_p.N_phi = 36;          % division of the angle for discretization

formation_limit = 0.05;

% Ball
ball_p.init_distance = 2;% [m] distance of the ball location from the center
ball_p.r = 0.1;          % [m] obstacle radius
ball_p.speed = 0.2;      % [m/s] obstacle speed
ball_p.w = deg2rad(1);   % [rad] uncertainty on the obstacle direction

% Simulation parameters----------------------------------------------------
video_flag = true;
steps_per_frame = 4; % take 1 frame every 3 steps of simulation

Ts = 5e-2;         % sampling time
sim_time = 18;     % total simulation time
n_sims = 18;       % number of simulations

names = {'Bob', 'Jet', 'Zoe', 'Tim', 'Lue', 'Hari', '007', 'Gaal', 'Tif',...
    'Hug', 'Mug', 'May'};

kp_formation = 2;
kp_obstacle = 2;
offset_cargo = 0.1;
%% objects initialization

cargo = RectangularCargo(st + center, cargo_p.center_mass, cargo_p.orientation,...
    cargo_p.dimensions);

pos_relative = randomStartingPositions(cargo_p.dimensions, division_starting_spots...
      , n_agents) - ones(2,1) .* cargo_p.dimensions / 2;
  
for i = 1:n_agents
    pos = cargo.relative2absolute(pos_relative(:,i));
    agents(i) = agent(names(i), pos, agent_p, cargo, map);
end

robots = flock(agents, cargo, Ts, 0);

ball_v = [cos(ball_direction); sin(ball_direction)] * ball_speed;
ball = Obstacle(ball_r, ball_starting_point, ball_v, Ts);

%% spread under the cargo
steps = fix(sim_time / Ts);
SUC_steps = 30;

last_d = zeros(robots.n_agents, 1);

h = figure();
h.Visible = 'off';
axis tight manual
ax = gca;
ax.NextPlot = 'replaceChildren';
GIF(steps) = struct('cdata',[],'colormap',[]);
v = VideoWriter('no_constraints.avi');
v.FrameRate = 10;
open(v);

% Spread under cargo (first phase)
disp('Starting: spread under cargo phase');
for i = 1:SUC_steps
    loadingBar(i, n_sims, 20, '#');
    robots.meetNeighbours();
    robots.computeVisibilitySets(); % no obstacle is present in this phase
    robots.computeVoronoiTessellationCargo(offset_cargo);
    robots.applyFarFromCenterMassDensity(5);
    robots.computeVoronoiCentroids();
    robots.moveToCentroids(kp_formation);

end
    
for i = 1:steps
    loadingBar(i, n_sims, 20, '#');
    robots.meetNeighbours();
    robots.sendScan(); % send scan
    robots.computeVisibilitySets(ball);
    robots.connectivityMaintenance();
    robots.computeVoronoiTessellationCargo(offset_cargo);
    
    robots.applyConstantDensity('Obstacle');
    robots.applyConstantDensity('Formation');
    
    robots.computeVoronoiCentroids();
    robots.moveToCentroids(kp_formation, kp_obstacle)
    ball.move();
    
    % video setup
    hold on         
    xlim([0,map_width]);
    ylim([0,map_height]);
    axis equal
    grid on
    show(map);
    %robots.plotVoronoiTessellationDetailed(1);
    robots.plot();
    %robots.plotCentroids();
    ball.plot();
    hold off
    
    GIF(i) = getframe(gcf);
    clf(h);
    writeVideo(v, GIF(i));

end
h.Visible = 'on';
close(v);