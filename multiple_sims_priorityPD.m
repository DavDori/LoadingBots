%% DODGE SIMULATION WITH MORE AGENTS AND CONSTRAINED MOVEMENT UNDER CARGO
clear all
close all
clc

%% SET UP

st = [0;0]; % starting position offset (applied to agents and cargo)
% map
map = png2BOMap('map_test_1.png',22);
map_width = map.XLocalLimits(2) - map.XLocalLimits(1);
map_height = map.YLocalLimits(2) - map.YLocalLimits(1);

% Cargo parameters
center = [map.XWorldLimits(2) / 2 ; map.YWorldLimits(2) / 2]; % [m]
cargo_p.center_mass = [0;0];        % [m]
cargo_p.dimensions =  [1.5; 1.5];   % [m]
cargo_p.orientation = pi/2;         % [rad]

% Agents
n_agents = 10;
division_starting_spots = 4;
agent_p.range = 1.0;            % [m] max observable range
agent_p.comm_range = 1.0;       % [m] max connection distance
agent_p.radius = 0.05;         % [m] hitbox of the agent
agent_p.N_rho = 40;           % division of the radius for discretization
agent_p.N_phi = 40;           % division of the angle for discretization
agent_p.max_speed = 0.5;     % [m/s] maximum speed reachable by the agent

% Ball
ball_p.init_distance = 2;% [m] distance of the ball location from the center
ball_p.r = 0.3;          % [m] obstacle radius [0.1, 0.2, 0.3]
ball_p.speed = 0.2;      % [m/s] obstacle speed
ball_p.w = deg2rad(1);   % [rad] uncertainty on the obstacle direction

% Simulation parameters----------------------------------------------------

Ts = 5e-2;         % sampling time
sim_time = 16;     % total simulation time 7,12,16
n_sims = 20;       % number of simulations

% Centroids gains ---------------------------------------------------------
% consider that the formation cell is always included in the obstacle
% Voroni cell
kp_formation = 6;   % formation centroid gain
kp_obstacle = 0.0;    % obstacle centroid gain

SUC_steps = 50;     % spread under cargo steps
offset_cargo = 0.1; % [m] offset from cargo shape where robots can go
offset_cargo_PD = 0.4;

bound = 0.05; % buonds to keep when in formation
hold_positions_factor = 0.2;

% attaching 
param_at.Kfor = 1;
param_at.Kobs = 1;
param_at.th = 0.05; % attach threshold

% detaching
param_dt.th = 0.8; % thershold for detach
density_in = 1;     % in angle range density multiplier
density_out = 0.3;  % out angle range density multiplier

% prioirty
Kp = 4; % importance of obstacle distance in priority
Kd = 7;   % importance of obstacle velocity in priority

min_p_COM = 0;                 % min priority bonus due to distance form COM
max_p_COM = param_dt.th * 0.5; % max priority bonus due to distance form COM

% agents starting positions -----------------------------------------------



names = {'Bob', 'Jet', 'Zoe', 'Tim', 'Lue', 'Hari', '007', 'Gaal', 'Tif',...
    'Hug', 'Mug', 'May','Emy', 'Jim', 'Pos', 'Ari', 'Sin', 'Cos', 'Mem'};

steps = fix(sim_time / Ts);




%% OBJECT INIT
cargo = RectangularCargo(st + center, cargo_p.center_mass, cargo_p.orientation,...
    cargo_p.dimensions);

%% MULTIPLE SIMULATIONS
% build array to take into account the successful simulations for each
% angle
success_dodge = ones(2, n_sims); % 1=no crashes, 0=crashes
success_dodge(1,:) = (2*pi/n_sims):(2*pi/n_sims):2*pi;

for j = 1:n_sims
    %loadingBar(j, n_sims, 20, '#');
    disp(strcat('simulation:_ ', string(j)));
    % each time resets positions and redefine the agents
    pos_relative = randomStartingPositions(cargo_p.dimensions, division_starting_spots...
          , n_agents) - ones(2,1) .* cargo_p.dimensions / 2;
    
    for i = 1:n_agents
        pos = cargo.relative2absolute(pos_relative(:,i));
        agents(i) = agent(names(i), pos, agent_p, cargo, map);
    end

    robots = flock(agents, cargo, Ts, 0);

    % obstacle movement direction
    % init_angle: [rad] angle at which the ball is located wtr the center
    ball_p.init_angle = success_dodge(1,j);   
    ball_p.dir = pi + ball_p.init_angle + rand(1) * deg2rad(ball_p.w); 
    ball_p.init_point = center + ball_p.init_distance *...
                         [cos(ball_p.init_angle); sin(ball_p.init_angle)];

    ball_p.v = [cos(ball_p.dir); sin(ball_p.dir)] * ball_p.speed;
    Ball = Obstacle(ball_p.r, ball_p.init_point, ball_p.v, Ts);

    % Spread under cargo (first phase)
    disp('Starting: spread under cargo phase');
    for i = 1:SUC_steps
        robots.meetNeighbours();
        robots.sendScan([]); 
        robots.computeVisibilitySets(); % no obstacle is present in this phase
        robots.liberalConnectivityMaintenance('All', 'All');
        robots.computeVoronoiTessellationCargo(offset_cargo);
        robots.applyFarFromCenterMassDensity(5);
        robots.applyConstantDensity('Obstacle');
        robots.computeVoronoiCentroids();
        robots.moveToCentroids(kp_formation);

    end
    disp('Spread under cargo phase concluded...');

    % PD algorithm
    % attach all agents 
    robots.attach();
    % set positions to hold
    hold_positions = robots.getAgentsPositions('All');
    % initialize the obstacle distances for PD priority with the maximunm value
    last_d = zeros(robots.n_agents, 1); % set to 0
    disp('Starting: dodging phase');
    for i = 1:steps

        robots.meetNeighbours();
        robots.sendScan(Ball); 

        robots.fixFormation('Attached'); % set bounds

        robots.computeVisibilitySets(Ball);

        % detached robots have just to stay in connectivity range
        %robots.connectivityMaintenance('Detached', 'All');
        robots.liberalConnectivityMaintenance('Detached', 'All');

        %detached must stay in the box limits
        robots.computeVoronoiTessellationCargo(offset_cargo_PD, 'Detached', 'All');
        % otherwise use
        %robots.computeVoronoiTessellation('Detached', 'All');

        robots.applyConstantDensity('Obstacle');

        % all robots should tend to return to the original position under cargo
        robots.applyMultiplePointsDensity(hold_positions, hold_positions_factor, 'All'); 

        robots.dodgeDensity(density_in, density_out, []);

        robots.computeVoronoiCentroids();

        % get priority for each agent. Agents in a sfae zone that want to
        % attach will have a small priority value, while agents in danger an
        % high priority value
        [priorityPD, new_d] = robots.priorityPD(Kp, Kd, last_d);
        % compute the priority in function of the distance from the center of
        % mass of the cargo
        priorityCOM = robots.priorityCOM(min_p_COM, max_p_COM, max(cargo.dimension));

        priority = priorityPD + priorityCOM;
        priority_filtered = priority;

        % return id of attacheable and detachable agents
        ids_detachable = robots.detachable();
        ids_attachable = robots.attachable();

        % calculate the centroid module for both attachable condition
        [m_obs_attach, m_for_attach] = robots.centroidsModule(1:robots.n_agents);
        drive_force = m_obs_attach + m_for_attach;
        
        [id_detachable] = detachRobotPriority(ids_detachable, priority, param_dt.th);

        % next step of simulation
        % follows the vectorial sum of both vectors defined by the centroids.
        % In some cases the obstacle should be already considered by the
        % formation cell but for non constant densities it has a different
        % weight
        robots.moveToCentroids(kp_formation, kp_obstacle, 'Detached');
        Ball.move();
        
        ids_at = robots.getAttached();
        ids_dt = robots.getDetached();
        
        if(isempty(id_detachable) == false)
            robots.detach(id_detachable);
        end
        [ids_equilibirum] = attachRobotEquilibrium(ids_attachable, drive_force,...
                            priority, param_at.th, param_dt.th);
        if(isempty(ids_equilibirum) == false)
            robots.attach(ids_equilibirum);
        end

        last_d = new_d;

        % critical feilure conditions
        hit = robots.hasBeenHit(Ball); % check if there has been a collision
        if(hit == true)
            disp('Critical failure: Collision!');
            success_dodge(2,j) = 0;
            break; % exit the simulation loop
        end
    end
    disp('Dodging phase concluded...');
end

%% Data analysis
disp(success_dodge);
