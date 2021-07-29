%% DODGE SIMULATION WITH MORE AGENTS AND CONSTRAINED MOVEMENT UNDER CARGO
clear all
close all
clc
%% SET UP
video_flag = true;
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
agent_p.range = 2;            % [m] max observable range
agent_p.comm_range = 5;       % [m] max connection distance
agent_p.radius = 0.1;         % [m] hitbox of the agent
agent_p.N_rho = 36;           % division of the radius for discretization
agent_p.N_phi = 36;           % division of the angle for discretization

% Ball
ball_p.init_angle = 0;   % [rad] angle at which the ball is located wtr the center
ball_p.init_distance = 2;% [m] distance of the ball location from the center
ball_p.r = 0.1;          % [m] obstacle radius
ball_p.speed = 0.4;      % [m/s] obstacle speed
ball_p.w = deg2rad(5);   % [rad] uncertainty on the obstacle direction
% Simulation parameters
Ts = 5e-2;
sim_time = 8;
slow_factor = 0.5;

kp_formation = 2;   % formation centroid gain
kp_obstacle = 4;    % obstacle centroid gain
SUC_steps = 30;     % spread under cargo steps
offset_cargo = 0.1; % [m] offset from cargo shape where robots can go

bound = 0.05; % buonds to keep when in formation
hold_positions_factor = 0.3;

% prioirty
Kp = 0.25; % importance of obstacle distance in priority
Kd = 1;   % importance of obstacle velocity in priority

alpha_com = 5; % convergance rate of the priority on center of mass distance
K_com = 0.10;  % importance of the priority on center of mass distance

% attaching 
param_at.Kfor = 1;
param_at.Kobs = 1;
param_at.th = 0.05; % attach threshold

% detaching
param_dt.th = 0.08; % thershold for detach
%% objects initialization

cargo = rect_load(st + center, cargo_p.center_mass, cargo_p.orientation,...
    cargo_p.dimensions);

w = 0.05; % perturbation scale factor
pos1 = center + [ 0.4;  0.4] + st + rand(2,1)*w;
pos2 = center + [ 0.4; -0.4] + st + rand(2,1)*w;
pos3 = center + [-0.4;  0.4] + st + rand(2,1)*w;
pos4 = center + [-0.4; -0.4] + st + rand(2,1)*w;
pos5 = center + [ 0.2;  0.2] + st + rand(2,1)*w;
pos6 = center + [-0.2; -0.2] + st + rand(2,1)*w;
pos7 = center + [-0.2;  0.2] + st + rand(2,1)*w;
pos8 = center + [ 0.2; -0.2] + st + rand(2,1)*w;

agents(1) = agent('Ali', pos1, agent_p, cargo, map);
agents(2) = agent('Sam', pos2, agent_p, cargo, map);
agents(3) = agent('Bob', pos3, agent_p, cargo, map);
agents(4) = agent('Kid', pos4, agent_p, cargo, map);
agents(5) = agent('Rip', pos5, agent_p, cargo, map);
agents(6) = agent('Fox', pos6, agent_p, cargo, map);
agents(7) = agent('Rob', pos7, agent_p, cargo, map);
agents(8) = agent('Est', pos8, agent_p, cargo, map);

robots = flock(agents, cargo, Ts, 0);

% obstacle movement direction
ball_p.dir = pi + ball_p.init_angle + rand(1) * deg2rad(ball_p.w); 
ball_p.init_point = center + ball_p.init_distance *...
                     [cos(ball_p.init_angle); sin(ball_p.init_angle)];
                 
ball_p.v = [cos(ball_p.dir); sin(ball_p.dir)] * ball_p.speed;
Ball = Obstacle(ball_p.r, ball_p.init_point, ball_p.v, Ts);

%% Video setup

steps = fix(sim_time / Ts);

if(video_flag == true)
    h = figure();
    h.Visible = 'off';
    axis tight manual
    ax = gca;
    ax.NextPlot = 'replaceChildren';
    v = VideoWriter('sim_PD.avi');
    v.FrameRate = fix(slow_factor * steps / sim_time);
    open(v);
end

%% Spread under cargo (first phase)

for i = 1:SUC_steps
    loadingBar(i, SUC_steps, 20, '#');
    robots.meetNeighbours();
    robots.computeVisibilitySets(); % no obstacle is present in this phase
    robots.computeVoronoiTessellationCargo(offset_cargo);
    robots.applyFarFromCenterMassDensity(5);
    robots.computeVoronoiCentroids();
    robots.moveToCentroids(kp_formation);
    
    if(video_flag == true)
        % video setup
        hold on         
        xlim([0,map_width]);
        ylim([0,map_height]);
        axis equal
        grid on
        show(map);
        %robots.plotVoronoiTessellationDetailed(3);
        robots.plot();
        robots.plotCentroids('Formation');
        Ball.plot();
        hold off

        frm = getframe(gcf);
        clf(h);
        writeVideo(v, frm);
    end
end
disp('Spread under cargo phase concluded...');

%% PD algorithm


% attach all agents 
robots.attach();
% set positions to hold
hold_positions = robots.getAgentsPositions('All');
% initialize the obstacle distances for PD priority with the maximunm value
last_d = zeros(robots.n_agents, 1) * agent_p.range; % set to 0

for i = 1:steps
    loadingBar(i, steps, 20, '#');
    
    robots.meetNeighbours();
    robots.sendScan(Ball); 
    
    robots.fixFormation('Attached'); % set bounds
            
    robots.computeVisibilitySets(Ball);
    % robots attached under cargo must maintain the formation, the upper
    % bound is set
    %robots.connectivityMaintenanceFF(bound, 'Attached', 'Attached');
    % detached robots have just to stay in connectivity range
    robots.connectivityMaintenance('Detached', 'All');
    
    %set the lower bound for the formation
    %robots.computeVoronoiTessellationFF(bound, 'Attached', 'Attached');
    
    %detached must stay in the box limits
    %robots.computeVoronoiTessellationCargo(offset_cargo, 'Detached', 'Detached');
    % otherwise use
    robots.computeVoronoiTessellation('Detached', 'Detached');
    
    robots.applyConstantDensity('Obstacle');
    
    % all robots should tend to return to the original position under cargo
    robots.applyMultiplePointsDensity(hold_positions, hold_positions_factor, 'All');
    % add 
    robots.applyFarFromCenterMassDensity(2, 'Detached');
    
    robots.computeVoronoiCentroids();
    
    % get priority for each agent. Agents in a sfae zone that want to
    % attach will have a small priority value, while agents in danger an
    % high priority value
    [priorityPD, new_d] = robots.priorityPD(Kp, Kd, last_d);
    % compute the priority in function of the distance from the center of
    % mass of the cargo
    priorityCOM = robots.priorityCOM(alpha_com, K_com);
    priority = priorityPD + priorityCOM;
    
    % return id of attacheable and detachable agents
    ids_detachable = robots.detachable();
    ids_attachable = robots.attachable();
    
    
    %robots.printWithAgentName(priority);
    
    % calculate the centroid module for both attachable and detachable
    [m_obs_attach, m_for_attach] = robots.centroidsModule(1:robots.n_agents);
   
    if(isempty(ids_detachable) == false) % there is at least a robot that can detach
        % select the one with higher priority. One agent at the time can
        % detach
        [val_detach, id] = max(priority(ids_detachable));
        
        if(val_detach > param_dt.th)
            robots.detach(id);
        end
    end
    
    if(isempty(ids_attachable) == false) % there is at least a robot that can detach
        
        drive_force = m_obs_attach + m_for_attach;
        % select agents that have a drive force smaller than the threshold,
        % meaning they are close to an equilibrium
        for j = ids_attachable
            if(drive_force(j) < param_at.th)
                robots.attach(j);
            end
        end
    end
    
    % next step of simulation
    % follows the vectorial sum of both vectors defined by the centroids.
    % In some cases the obstacle should be already considered by the
    % formation cell but for non constant densities it has a different
    % weight
    robots.moveToCentroids(kp_formation, kp_obstacle, 'Detached');
    Ball.move();
    
    if(video_flag == true)
        % video setup
        hold on         
        xlim([0,map_width]);
        ylim([0,map_height]);
        axis equal
        grid on
        show(map);
        %robots.plotVoronoiTessellationDetailed(3);
        robots.plot();
        robots.plotCentroids('Obstacle');
        robots.plotCentroids('Formation');
        Ball.plot();
        hold off

        frm = getframe(gcf);
        clf(h);
        writeVideo(v, frm);
    end
    
    last_d = new_d;
    
    % critical feilure conditions
    hit = robots.hasBeenHit(Ball); % check if there has been a collision
    if(hit == true)
        disp('Critical failure: Collision!');
        break; % exit the simulation loop 
    end
end

if(video_flag == true)
    h.Visible = 'on';
    close(v);
end