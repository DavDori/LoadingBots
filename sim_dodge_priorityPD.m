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

% ball
ball_starting_point = [0.5; 0.5];
ball_r = 0.1; % [m] obstacle radius
ball_speed = 0.5; % [m/s] obstacle speed
ball_direction = deg2rad(10); % obstacle direction

% Simulation parameters
Ts = 1e-1;
sim_time = 2;
steps = 80;

kp_formation = 2; % formation centroid gain
kp_obstacle = 3;  % obstacle centroid gain

offset_cargo = 0.1; %[m]

bound = 0.05; % buonds to keep when in formation
hold_positions_factor = 0.3;

% attaching 
param_at.Kp = 1;
param_at.Kd = 1;
param_at.th = 0.20;

% detaching
param_dt.Kp = 0.5;   % importance of obstacle distance in priority
param_dt.Kd = 1;   % importance of obstacle velocity in priority
param_dt.th = 0.25; % thershold for detach
%% objects initialization

cargo = rect_load(st + center, center_mass, orientation, dimensions);

s = 0.0; % perturbation scale factor
pos1 = [2.840878; 1.591456] + st + randn(2,1)*s;
pos2 = [2.834078; 0.421744] + st + randn(2,1)*s;
pos3 = [2.112834; 0.415857] + st + randn(2,1)*s;
pos4 = [2.112496; 1.586409] + st + randn(2,1)*s;
pos5 = [2.036223; 1.015489] + st + randn(2,1)*s;
pos6 = [2.928917; 1.020818] + st + randn(2,1)*s;
pos7 = center + randn(2,1)*s;
agents(1) = agent('Mulan', pos1, param, cargo, map);
agents(2) = agent('Pluto', pos2, param, cargo, map);
agents(3) = agent('Gerlad',pos3, param, cargo, map);
agents(4) = agent('Leila', pos4, param, cargo, map);
agents(5) = agent('Samuel',pos5, param, cargo, map);
agents(6) = agent('Anakin',pos6, param, cargo, map);
agents(7) = agent('Robin', pos7, param, cargo, map);

robots = flock(agents, cargo, Ts, 0);

ball_v = [cos(ball_direction); sin(ball_direction)] * ball_speed;
ball = Obstacle(ball_r, ball_starting_point, ball_v, Ts);


    
%% PD algorithm 
if(video_flag == true)
    h = figure();
    h.Visible = 'off';
    axis tight manual
    ax = gca;
    ax.NextPlot = 'replaceChildren';
    v = VideoWriter('sim_PD.avi');
    v.FrameRate = 5;
    open(v);
end

% attach all agents 
robots.attach();
% set positions to hold
hold_positions = robots.getAgentsPositions('All');
% initialize the obstacle distances for PD priority with the maximunm value
obs_dist = zeros(robots.n_agents, 1) * param.range;

for i = 1:steps
    
    robots.meetNeighbours();
    robots.sendScan(ball); 
    
    robots.fixFormation('Attached'); % set bounds
            
    robots.computeVisibilitySets(ball);
    % robots attached under cargo must maintain the formation, the upper
    % bound is set
    robots.connectivityMaintenanceFF(bound, 'Attached', 'Attached');
    % detached robots have just to stay in connectivity range
    robots.connectivityMaintenance('Detached', 'All');
    
    %set the lower bound for the formation
    robots.computeVoronoiTessellationFF(bound, 'Attached', 'Attached');
    
    %detached must stay in the box limits
    robots.computeVoronoiTessellationCargo(offset_cargo, 'Detached', 'Detached');
    
    robots.applyConstantDensity('Obstacle');
    
    % all robots should tend to return to the original position under cargo
    robots.applyMultiplePointsDensity(hold_positions, hold_positions_factor, 'All');
    
    % selecct robots that are underneath the cargo
    id_attach = robots.attachable();
    [d1,d2] = robots.centroidsModule(id_attach);
    [id_detach, val_detach, obs_dist] = robots.priorityRankingPD(param_dt.Kp, param_dt.Kd, obs_dist);
    
    if(isempty(id_detach) == false) % there is at least a robot that can detach
        if(val_detach > param_dt.th) % agent wants to move
            robots.detach(id_detach);
        end
    end
    if(isempty(id_attach) == false) % there is at least a robot that can detach
        robots.attach(id_detach);
    end
    fprintf('time %d\n', i * Ts);
    robots.printStatus();
    % next step of simulation
    robots.moveToCentroids(kp_formation, kp_obstacle, 'Detached');
    ball.move();
    
    if(video_flag == true)
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

        frm = getframe(gcf);
        clf(h);
        writeVideo(v, frm);
    end
end
if(video_flag == true)
    h.Visible = 'on';
    close(v);
end