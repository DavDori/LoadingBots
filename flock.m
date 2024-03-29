classdef flock < handle
    %FLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        agents agent
        cargo RectangularCargo
        n_agents (1,1) double {mustBeNumeric}
        formation_factor (1,1) double {mustBeNumeric}
    end
    
    methods
        function obj = flock(agents_array, box, sampling_time, xi)
            %FLOCK 
            obj.agents = agents_array;
            obj.n_agents = length(agents_array);
            obj.cargo = box;
            obj.formation_factor = xi;
            for a = obj.agents
                a.Ts = sampling_time;
            end
        end
        
        
        function p = getPositions(obj)
            % return the position of every agent
            n = length(obj.agents);
            p = zeros(n, 2);
            for i = 1:n
                p(i,:) = obj.agents(i).position';
            end
        end
        
        
        function attach(obj, set)
            % attach the selected agents on the board
            if(nargin < 2)
                set = 1:obj.n_agents;
            end
            for i = set
                obj.agents(i).attach(); 
            end
        end
        
        function saveCargoPosition(obj, set)
            % attach the selected agents on the board
            if(nargin < 2)
                set = 1:obj.n_agents;
            end
            for i = set
                obj.agents(i).saveCargoPosition(); 
            end
        end
        
        function detach(obj, set)
            % detach the selected agents on the board
            if(nargin < 2)
                set = 1:obj.n_agents;
            end
            for i = set
                obj.agents(i).detach(); 
            end
        end
        
        
        function moveAgent(obj, index, velocity)
            % move a specific agent
            if(index < obj.n_agents)
                obj.agents(index).move(velocity);
            end
        end
        
        
        function flag = checkBalance(obj, id_agents)
            % check if the load should be falling
            points = zeros([length(id_agents), 2]);
            n = 0;
            for i = id_agents
                if(obj.agents(i).attached == true) % considers only attached agents
                    n = n + 1;
                    points(n,:) = obj.agents(i).position';
                end
            end
            flag = obj.cargo.isBalanced(points(1:n,:));
        end
        
        
        function sendScan(obj, obs)
            % every agent send a message to its neighbours containing name
            % and its absolute coordinates
            for i = 1:obj.n_agents
                if(nargin > 1)
                    scan = obj.agents(i).scan(obs);
                else
                    scan = obj.agents(i).scan([]);
                end
                for j = 1:obj.n_agents
                    if(i ~= j)
                        obj.agents(i).sendScan(obj.agents(j), scan);
                    end
                end
            end
        end
        
        
        function meetNeighbours(obj)
            % send message to each agent and decode it so that every agent
            % will know its neighbours position.
            
            clearAllCommsData(obj);
            sendNamePosition(obj);
            for a = obj.agents
                a.decodeTextIn();
            end
        end
        
        
        function sendNamePosition(obj)
            % every agent send a message to its neighbours containing name
            % and its absolute coordinates
            for i = 1:obj.n_agents
                for j = [1:i-1, i+1:obj.n_agents]
                    if(obj.agents(i).attached == true)
                        att = ',A';
                    else
                        att = ',D';
                    end
                    % string to send
                    mex = strcat('N', obj.agents(i).name, ...
                                 ',X', num2str(obj.agents(i).position(1)),...
                                 ',Y', num2str(obj.agents(i).position(2)), att,';');
                    obj.agents(i).sendMessage(obj.agents(j), mex);
                end
            end
        end
        
        
        function clearAllCommsData(obj)
            % to perform every step, clear the communication data such as
            % neighbours location and scans
            for a = obj.agents
                a.clearComms();
                a.clearScan();
                a.clearNeighbours();
            end
        end
        
        function computeVoronoiTessellationCargo(obj, offset, type_AA, type_NB)
            % compute the Voronoi tessellation of a discretization of the
            % nearby area for every agent in the flock considering the
            % perimeter of the cargo
            if(nargin < 3)
                type_AA = 'All';
            end
            if(nargin < 4)
                type_NB = 'All';
            end
            ids = obj.agentSelector(type_AA);
            
            for i = ids
                obj.agents(i).computeCellCollisionAvoidance(type_NB);
                obj.agents(i).applyVoronoiCargoLimits(offset);
            end
        end
        
        
        function computeVoronoiTessellation(obj, type_AA, type_NB)
            % compute the Voronoi tessellation of a discretization of the
            % nearby area for every agent in the flock. Moreover it takes 
            % into account collision avoidance
            if(nargin < 3)
                type_AA = 'All';
            end
            if(nargin < 4)
                type_NB = 'All';
            end
            ids = obj.agentSelector(type_AA);
            
            for i = ids
                obj.agents(i).computeCellCollisionAvoidance(type_NB);
            end
        end
        
        
        function ids = agentSelector(obj, type)
            % selects agents given a type identifier 
            if(strcmp(type, 'Attached') == true)
                ids = getAttached(obj);
            elseif(strcmp(type, 'Detached') == true)
                ids = getDetached(obj);
            elseif(strcmp(type, 'All') == true)
                ids = 1:obj.n_agents;
            else
                error('wrong type of agent selected! Specify Attached, All or Detached as type')
            end
        end
        
        
        function wp = getCurrentWayPoints(obj)
            % return list of way_points for each agent
            wp = zeros(obj.n_agents, 2);
            for i = 1:obj.n_agents
                wp(i,:) = obj.agents(i).way_point;
            end
        end
        
        
        function computeVoronoiTessellationFF(obj, relax_factor, type_AA, type_NB)
            % compute the Voronoi tessellation of a discretization of the
            % nearby area for every agent in the flock. Moreover it takes 
            % into account formation facor. By setting type to attached or
            % detached, it is possible to select different agents to act
            % with 'type_AA' and the neighbours to consider with 'type_NB'
            if(nargin < 3)
                type_AA = 'All';
            end
            if(nargin < 4)
                type_NB = 'All';
            end
            ids = obj.agentSelector(type_AA);
            
            for i = ids
                obj.agents(i).computeCellFormation(relax_factor, type_NB); 
            end
        end
        
        
        function computeVisibilitySets(obj, obs)
            % compute the visibility set for every agent
            if nargin < 2
                obs = [];
            end
                
            for a = obj.agents
                a.computeVisibilitySet(obs); 
            end
        end
        
        
        function connectivityMaintenanceFF(obj, relax_factor, type_AA, type_NB)
            % modify the voronoi cells of every agent corresponding to the specified type,
            % to allow connectivity maintenance for fixed formation
            if(nargin < 3)
                type_AA = 'All';
            end
            if(nargin < 4)
                type_NB = 'All';
            end
            ids = obj.agentSelector(type_AA);
            
            for i = ids
                obj.agents(i).applyConnectivityMaintenance(relax_factor, type_NB);
            end
        end
        
        function liberalConnectivityMaintenance(obj, type_AA, type_NB)
            % less strict version of connectivity maintenace that allow for
            % an agnet to loose a neighbour but not the last one
            if(nargin < 2)
                type_AA = 'All';
            end
            if(nargin < 3)
                type_NB = 'All';
            end
            ids = obj.agentSelector(type_AA);
            
            for i = ids
                obj.agents(i).applyLiberalConnectivityMaintenance([], type_NB);
            end
        end
        
            
        function connectivityMaintenance(obj, type_AA, type_NB)
            % modify the voronoi cells of every agent corresponding to the specified type,
            % to allow connectivity maintenance
            if(nargin < 2)
                type_AA = 'All';
            end
            if(nargin < 3)
                type_NB = 'All';
            end
            ids = obj.agentSelector(type_AA);
            
            for i = ids
                obj.agents(i).applyConnectivityMaintenance([], type_NB);
            end
        end
        
        
        function centroids = computeVoronoiCentroids(obj, type)
            % compute the centroid of every agent voronoi cell. It's
            % possible to calculate the centroid relative to the formation
            % or obstacle
            centroids = zeros(obj.n_agents, 2);
            if(nargin < 2)
                for i = 1:obj.n_agents
                    centroids(i, 1:2) = obj.agents(i).getFormationCentroid();
                end
            elseif(strcmp(type, 'Formation'))
                for i = 1:obj.n_agents
                    centroids(i, 1:2) = obj.agents(i).getFormationCentroid();
                end
            elseif(strcmp(type, 'Obstacle'))
                for i = 1:obj.n_agents
                    centroids(i, 1:2) = obj.agents(i).getObstacleCentroid();
                end
            elseif(strcmp(type, 'Both'))
                centroids = zeros(obj.n_agents, 4); % redefine and change size
                for i = 1:obj.n_agents
                    centroids(i, 1:2) = obj.agents(i).getFormationCentroid();
                    centroids(i, 3:4) = obj.agents(i).getObstacleCentroid();
                end
            else
                error(strcat('Error: wrong type: ', type, 'of voronoi cell selected'))
            end
        end
        
        
        % METHODS: voronoi cell density -----------------------------------
        
        function applyFarFromCenterMassDensity(obj, K, type)
            % for every robot of specified type, apply the far from center 
            % of mass density on the Voronoi cell
            if(nargin < 3)
                ids = obj.agentSelector('All');
            else
                ids = obj.agentSelector(type);
            end
            
            for i = ids
                obj.agents(i).applyVoronoiFarFromCargoDensity(K);
            end
        end
        
        
        function applyWayPointDensity(obj, sf, type)
            % for every robot apply an exponential density function
            % centered on its way point on the voronoi cell. 'sf' is the
            % spread factor of the points
            if(nargin < 3)
                ids = obj.agentSelector('All');
            else
                ids = obj.agentSelector(type);
            end
            
            for i = ids
                obj.agents(i).applyVoronoiWayPointDensity(sf);
            end
        end
        
        
        function applyMultiplePointsDensity(obj, points, sf, type)
            % for every robot apply an exponential density function
            % centered on a point which is given as input. 'sf' is the
            % spread factor of the point
            if(nargin < 4)
                ids = obj.agentSelector('All');
            else
                ids = obj.agentSelector(type);
            end
            
            for i = ids
                obj.agents(i).applyVoronoiPointDensity(points(i,:), sf);
            end
        end
        
        
        function applyMultiplePointsDensityLocal(obj, points, sf, type)
            % for every robot apply an exponential density function
            % centered on a point which is given as input. 'sf' is the
            % spread factor of the point
            if(nargin < 4)
                ids = obj.agentSelector('All');
            else
                ids = obj.agentSelector(type);
            end
            
            for i = ids
                obj.agents(i).applyVoronoiPointDensityLocal(points(i,:), sf);
            end
        end
        
        
        function applySinglePointDensity(obj, point, sf, type)
            % for every robot apply an exponential density function
            % centered on a common point on the voronoi cell. 'sf' is the
            % spread factor of the point
            if(nargin < 4)
                ids = obj.agentSelector('All');
            else
                ids = obj.agentSelector(type);
            end
            
            for i = ids
                obj.agents(i).applyVoronoiPointDensity(point, sf);
            end
        end
        
        
        function applySinglePointDensityLocal(obj, point, sf, type)
            % for every robot apply an exponential density function
            % centered on a common point on the voronoi cell. 'sf' is the
            % spread factor of the point
            if(nargin < 4)
                ids = obj.agentSelector('All');
            else
                ids = obj.agentSelector(type);
            end
            
            for i = ids
                obj.agents(i).applyVoronoiPointDensityLocal(point, sf);
            end
        end
        
        
        function applyConstantDensity(obj, type_cell, type_agents)
            % for every robot apply a constant density at its Voronoi cell
            % density
            if(nargin < 3)
                ids = obj.agentSelector('All');
            else
                ids = obj.agentSelector(type_agents);
            end
            
            if(nargin < 2)
                for i = ids
                    obj.agents(i).formation_VC.addConstantDensity();
                end
            elseif(strcmp(type_cell, 'Formation'))
                for i = ids
                    obj.agents(i).formation_VC.addConstantDensity();
                end
            elseif(strcmp(type_cell, 'Obstacle'))
                for i = ids
                    obj.agents(i).obstacle_VC.addConstantDensity();
                end
            else
                error(strcat('Error: wrong type: ', type_cell, 'of voronoi cell selected for constant density'))
            end
        end
        
        
        % MOTION PLANNING METHODS -----------------------------------------
        
        function setWayPoints(obj, dest, type)
            % given the destination of the cargo, compute the waypoints of
            % each agent
            if(nargin < 3)
                ids = obj.agentSelector('All');
            else
                ids = obj.agentSelector(type);
            end
            
            for i = ids
                % calculate the destination for each agent
                obj.agents(i).computeWayPoint(dest');
            end
        end
        
        
        function moveToCentroids(obj, kp_formation, kp_obstacle, type)
            % move all the agents in direction of their centroids for a
            % sampling time.
            if(nargin < 4)
                type = 'All';
            end
            ids = obj.agentSelector(type);
            
            c_formation = kp_formation * obj.computeVoronoiCentroids('Formation');
            if (nargin >= 3)
                c_obstacle  = kp_obstacle  * obj.computeVoronoiCentroids('Obstacle');
            else
                c_obstacle = zeros(size(c_formation));
            end
            
            % can be done because both are in the relative ref frame 
            c = c_obstacle + c_formation; 
            
            for i = ids
                centroid = c(i,:);
                % set kp as 1 to keep the driven gain dependent on the two kps
                obj.agents(i).moveToCentroid(1, centroid);  
            end
        end
        
        
        function spreadUnderCargo(obj, steps, offset, kd, obs)
            % distributed maximum coverage application
            % update the position of the neighbours
            if nargin < 5
                obs = [];
            end
            
            for i = 1:steps
                obj.meetNeighbours();
                obj.computeVisibilitySets(obs);
                obj.computeVoronoiTessellationCargo(offset);
                obj.applyFarFromCenterMassDensity();
                obj.computeVoronoiCentroids();
                obj.moveToCentroids(kd);
            end
        end
        
        
        function moveFormation(obj, formation_limit, k_d, k_WP, max_i, obs)
            % move the flock to a target position keeping a formation limit
            % error with a fixed formation. k_d parameter determine the
            % proportional gain in the movement towards the centroid and
            % max_i the maximum number of iteations. k_WP is the density
            % factor applied to the point the robots have to reach.
            exit = false;
            e = 0.01; % distance from waypoints that every agent has to reach
            n = 1;
            if nargin < 6
                obs = [];
            end
            while(exit == false && n < max_i)
                obj.meetNeighbours();
                obj.sendScan();
                % voroni cell 
                obj.computeVisibilitySets(obs);
                obj.connectivityMaintenance(formation_limit);
                obj.computeVoronoiTessellationFF(formation_limit);
                obj.applyWayPointDensity(k_WP);
                obj.computeVoronoiCentroids();
                % movement
                obj.moveToCentroids(k_d);
                %obj.fixFormation();
                reached = obj.areWayPointsReached(e);
                exit = all(reached);
                n = n + 1;
            end
            if(n >= max_i)
                waring('Convergance no achieved');
            end
        end
            
        
        function waypoints = setTrajectory(obj, cmds)
            % calculate the waypoints given a set of commands. The motion
            % is modeled as a unicycle and the starting position is the
            % current position of the cargo center. The output is a
            % sequence of n points
            % Note: cmds structure: [v, w] and time is considered as 1 s
            n = size(cmds,1); % rows
            waypoints = zeros(n, 3); % [x,y,theta]
            prev = [obj.agents(1).cargo.center; obj.agents(1).cargo.orientation]'; % initial pose
            for i = 1:n
                waypoints(i,:) = prev + unicycleModel(cmds(i,:), prev)';
                prev = waypoints(i,:);
            end
        end
        
        
        function fixFormation(obj, type)
            % should be computed after the flock has spread under the
            % cargo. It sets the bound to keep to the robots
            if(nargin < 2)
                ids = obj.agentSelector('All');
            else
                ids = obj.agentSelector(type);
            end
            
            for i = ids
                if(obj.agents(i).attached == false)
                    % if the agent is not attached, a warning is thrown
                    fprintf('WARNING: %s is not attached \n', obj.agents(i).name); 
                end
                obj.agents(i).fixBounds();
            end
        end
              
        
        function pos = getAgentsPositions(obj, type)
            % build a nx2 matrix with the agents positions
            if(nargin < 3)
                ids = obj.agentSelector('All');
            else
                ids = obj.agentSelector(type);
            end
            pos = zeros(length(ids),2);
            n = 0;
            for i = ids
                n = n + 1;
                pos(n,:) = obj.agents(i).position';
            end
        end
        
        
        function reached = areWayPointsReached(obj, error, type)
            % for every agent if they have reached their waypoint
            if(nargin < 3)
                ids = obj.agentSelector('All');
            else
                ids = obj.agentSelector(type);
            end
            
            reached = false(length(ids), 1); 
            n = 0;
            for i = ids
                n = n + 1;
                reached(n) = obj.agents(i).isWayPointReached(error);
            end
        end
        
        
        function setDensityAngles(obj, angles_range, v_in, v_out, ids, type)
            % se density inside the angles_range of each agent as v_in and
            % outside as v_out
            for i = ids
                obj.agents(i).setDensityAngle(angles_range(i,:), v_in, v_out, type);
            end
        end
        
        function dodgeDensity(obj, v_in, v_out, ids)
            % splits in halt the density along the obstacle centroid axis
            % and change its values
            if(isempty(ids) == true) % takes all the detached agents
                ids = obj.getDetached();
            end
            for i = ids
                angle_range = obj.agents(i).detach_angle;
                obj.agents(i).setDensityAngle(angle_range, v_in, v_out, 'Formation');
            end
        end
        
        function dodgeDensityDyn(obj, v_in, v_out, ids)
            % splits in halt the density along the obstacle centroid axis
            % and change its values dynamically
            if(isempty(ids) == true) % takes all the detached agents
                ids = obj.getDetached();
            end
            for i = ids
                alpha = obj.agents(i).getObstacleCentroidAngle();
                % only if the centroid has big enough module the angle is
                % considered
                if(isempty(alpha) == false)
                    angle_range = [alpha, alpha + pi]; 
                    obj.agents(i).setDensityAngle(angle_range, v_in, v_out, 'Formation');
                end
            end
        end
        
        % METHODS: priority -----------------------------------------------
        
        function p = priorityP(obj, Kp)
            % compute the proportional priority for each robot 
            p = zeros(length(obj.agents),1);
            for i = 1:length(obj.agents)
                [p(i), ~] = obj.agents(i).computePriorityP(Kp);
            end
        end
        
        
        function [p,d] = priorityPD(obj, Kp, Kd, last_d)
            % compute the proportional and derivative priority for each robot.
            % to compute the derivative term, the last distance readings
            % for each agent is needed 
            
            d = zeros(size(last_d));
            p = zeros(obj.n_agents, 1);
            for i = 1:obj.n_agents
                [p(i), d(i)] = obj.agents(i).computePriorityPD(Kp, Kd, last_d(i));
            end
        end
        
        
        function ids = getAttached(obj)
            % return the ids of all attached agents
            ids = zeros(1,obj.n_agents);
            index = 0;
            for i = 1:obj.n_agents
                if(obj.agents(i).attached == true)
                    index = index + 1;
                    ids(index) = i;
                end
            end
            ids = ids(1:index);
        end
        
        function ids = getDetached(obj)
            % return the ids of all detahced agents
            ids = zeros(1, obj.n_agents);
            index = 0;
            for i = 1:obj.n_agents
                if(obj.agents(i).attached == false)
                    index = index + 1;
                    ids(index) = i;
                end
            end
            ids = ids(1:index);
        end
        
        
        function ids = detachable(obj)
            % check if the agents detachment is going to cause critical
            % failure
            
            attached_ids = obj.getAttached();
            ids = zeros(1, length(attached_ids));
            index = 0;
            for i = attached_ids
                % don't consider itsself in the balance
                mates = attached_ids(attached_ids ~= i);
                
                if(checkBalance(obj, mates) == true)
                    index = index + 1;
                    ids(index) = i;
                end
            end
            ids = ids(1:index);
        end
        
        
        function ids = attachable(obj)
            % check if the agents have escaped a problematic situation and
            % can attach to the cargo again. 
            % param is a struct with priority parameters.
            
            detached_ids = obj.getDetached();
            ids = zeros(1, length(detached_ids));
            index = 0;
            % check only detached agents
            for i = detached_ids
                % check if the agent is inside the rectangle
                if(obj.agents(i).isInsideRectLoad() == true) 
                    index = index + 1;
                    ids(index) = i;
                end
            end
            ids = ids(1:index);
        end
        
        
        function [id,val] = priorityRankingP(obj, Kp)
            % return the id of the agent with higher priority that can move
            % using priority P
            p = obj.priorityP(Kp);
            p = p + 0.01; % small perturbation
            ids_d = obj.detachable();
            p_to_move = p(ids_d); % take only agents that can detach
            if (sum(abs(p_to_move)) == 0)
                id  = [];
                val = [];
            else
                [val,id] = max(p_to_move);
            end
        end
        
        
        function [id, val, new_d] = priorityRankingPD(obj, Kp, Kd, last_d)
            % return the id of the agent with higher priority that can move
            % using priority PD
            [p, new_d] = obj.priorityPD(Kp, Kd, last_d);
            ids_d = obj.detachable();
            p_to_move = p(ids_d); % take only agents that can detach
            if (sum(abs(p_to_move)) == 0)
                id  = [];
                val = [];
            else
                % between every agent that can move take the one with
                % higher priority
                [val,id] = max(p_to_move);
            end
        end
        
        
        function d = getDistancesFromCOM(obj, ids)
            % for the selected agents get the distance from the COM
            if(nargin < 2)
                ids = 1:obj.n_agents;
            end
            d = zeros(size(ids'));
            n = 0;
            for i = ids
                n = n + 1;
                d(n) = obj.agents(i).getDistanceFromCOM();
            end
        end
        
        
        function pCOM = priorityCOM(obj, min_p, max_p, max_d_COM)
            % compute the distance from the COM, the closest to 0 the
            % higher, meaning that the agent is a bit useless.

            d = getDistancesFromCOM(obj);
            m = (min_p - max_p) / max_d_COM;
            pCOM = m * d + max_p; 
        end
        
        
        function [m_obs, m_for] = centroidsModule(obj, ids)
            if(isempty(ids) == true)
                ids = 1:obj.n_agents;
            end
            m_obs = zeros(size(ids'));
            m_for = zeros(size(ids'));
            n = 0;
            if(isempty(ids) == false)
                for i = ids
                    n = n + 1;
                    [m_obs(n), m_for(n)] = obj.agents(i).centroidsModule();
                end
            end
        end
        
        
        function [m] = centroidModule(obj, ids)
            % calculate the module of the sum of the two centriods
            m = zeros(size(ids));
            n = 0;
            if(isempty(ids) == false)
                for i = ids
                    n = n + 1;
                    m(n) = obj.agents(i).centroidModule();
                end
            end
        end
        
        
        % METHODS: auxiliary      -----------------------------------------
        
        function hit = hasBeenHit(obj, obstacle)
            % check if an agent has been hit by an obstacle
            hit = false;
            for a = obj.agents
                hit = hit || a.checkHit(obstacle);
            end
        end
        
        % METHODS: representation -----------------------------------------
        
        
        function printWithAgentName(obj, ary)
            for i = 1:obj.n_agents
                disp(strcat(obj.agents(i).name, string(ary(i))));
            end
        end
        
        
        function plot(obj, plot_name_flag)
            % representation in a 2D plane of the agents and the load
            hold on
            axis equal
            for a = obj.agents
                a.plot(plot_name_flag);
            end
            hold off
        end
        
        function plotVoronoiTessellation(obj)
            % plot the limit perimeter of the Voronoi cells of every robot
            hold on
            for a = obj.agents
                a.formation_VC.plotFast(a.position, [rand,rand,rand]); 
            end
            hold off
        end
        
        
        function plotVoronoiTessellationDetailed(obj, step, res, ids)
            % plot the Voronoi cells of every robot considering every point
            % of the cell
            if nargin < 4
                ids = 1:obj.n_agents;
            end
            hold on
            for i = ids
                obj.agents(i).plotVoronoiCellDetailed(step, res); 
            end
            hold off
        end
        
        
        function plotVTObstacle(obj, step)
            % plot the Voronoi cells of every robot considering every point
            % of the cell
            hold on
            for a = obj.agents
                a.plotVoronoiCellDetailed(step, 'Obstacle'); 
            end
            hold off
        end
        
        
        function plotCentroids(obj, ids, type)
            hold on
            if(isempty(ids) == true)
                ids = 1:obj.n_agents;
            end
            for i = ids
                if(nargin < 3)
                    c = obj.agents(i).position + obj.agents(i).formation_VC.centroid;
                    plot([obj.agents(i).position(1), c(1)],[obj.agents(i).position(2), c(2)],'-r');
                    plot(c(1), c(2), 'ro');
                elseif(strcmp(type, 'Formation'))
                    c = obj.agents(i).position + obj.agents(i).formation_VC.centroid;
                    plot([obj.agents(i).position(1), c(1)],[obj.agents(i).position(2), c(2)],'-r');
                    plot(c(1), c(2), 'ro');
                elseif(strcmp(type, 'Obstacle'))
                    c = obj.agents(i).position + obj.agents(i).obstacle_VC.centroid;
                    plot([obj.agents(i).position(1), c(1)],[obj.agents(i).position(2), c(2)],'-b');
                    plot(c(1), c(2), 'bo');
                else
                    error(strcat('Error: wrong type: ', type, 'of voronoi cell in plot centroid'))
                end
            end
            hold off
        end
        
        
        function plotAgentsPath(obj, path)
            hold on
            for a = obj.agents
                path_agent = zeros(size(path) + [1,0]);
                path_agent(1,:) = [a.position', 0];
                for i = 1:size(path,1) % for every point in the path
                    path_agent(i + 1,:) = [a.computeWayPoint(path(i,:)')', 0];
                end
                plotPath(path_agent);
            end
            hold off
        end
        
        
        function print(obj)
            % print basic info on the agents
            for a = obj.agents
                a.print();
                fprintf('\n');
            end
        end
        
        
        function printStatus(obj)
            % print basic info on the agents
            for a = obj.agents
                a.printStatus();
                fprintf('\n');
            end
        end
        
        
        function set.formation_factor(obj, xi)
            obj.formation_factor = xi;
        end
    end
end

