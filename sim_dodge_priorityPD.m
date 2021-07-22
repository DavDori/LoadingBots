%% DODGE SIMULATION WITH MORE AGENTS AND CONSTRAINED MOVEMENT UNDER CARGO
clear all
close all
clc
%% SET UP
video_flag = false;
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
param.range = 2;            % [m] max observable range
param.comm_range = 5;       % [m] max connection distance
param.radius = 0.1;         % [m] hitbox of the agent
param.N_rho = 36;           % division of the radius for discretization
param.N_phi = 36;           % division of the angle for discretization

% Ball
Ball_starting_point = [1; 0.8];
Ball_r = 0.1; % [m] obstacle radius
Ball_speed = 0.4; % [m/s] obstacle speed
Ball_direction = deg2rad(10); % obstacle direction

% Simulation parameters
Ts = 1e-2;
sim_time = 8;
slow_factor = 0.5;

kp_formation = 2; % formation centroid gain
kp_obstacle = 3;  % obstacle centroid gain

offset_cargo = 0.1; %[m]

bound = 0.05; % buonds to keep when in formation
hold_positions_factor = 0.3;

% prioirty
Kp = 0.5; % importance of obstacle distance in priority
Kd = 1;   % importance of obstacle velocity in priority

alpha_com = 5; % convergance rate of the priority on center of mass distance
K_com = 0.15; % importance of the priority on center of mass distance

% attaching 
param_at.Kfor = 1;
param_at.Kobs = 1;
param_at.th = 0.1;

% detaching
param_dt.th = 0.2; % thershold for detach
%% objects initialization

cargo = rect_load(st + center, center_mass, orientation, dimensions);

s = 0.0; % perturbation scale factor
pos1 = center + [0.4 ; 0.7 ] + st + randn(2,1)*s;
pos2 = center + [0.4 ; -0.7] + st + randn(2,1)*s;
pos3 = center + [-0.4; 0.7 ] + st + randn(2,1)*s;
pos4 = center + [-0.4; -0.7] + st + randn(2,1)*s;
pos5 = center + [ 0; 0] + st + randn(2,1)*s;
agents(1) = agent('Mulan', pos1, param, cargo, map);
agents(2) = agent('Pluto', pos2, param, cargo, map);
agents(3) = agent('Gerald',pos3, param, cargo, map);
agents(4) = agent('Leila', pos4, param, cargo, map);
agents(5) = agent('Samuel',pos5, param, cargo, map);

robots = flock(agents, cargo, Ts, 0);

Ball_v = [cos(Ball_direction); sin(Ball_direction)] * Ball_speed;
Ball = Obstacle(Ball_r, Ball_starting_point, Ball_v, Ts);


    
%% PD algorithm
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

% attach all agents 
robots.attach();
% set positions to hold
hold_positions = robots.getAgentsPositions('All');
% initialize the obstacle distances for PD priority with the maximunm value
last_d = ones(robots.n_agents, 1) * param.range;

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
    robots.computeVoronoiTessellationCargo(offset_cargo, 'Detached', 'Detached');
    
    robots.applyConstantDensity('Obstacle');
    
    % all robots should tend to return to the original position under cargo
    robots.applyMultiplePointsDensity(hold_positions, hold_positions_factor, 'All');
    
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