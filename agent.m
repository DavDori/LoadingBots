classdef agent < handle
    %AGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name % id of the agent
        position (2,1) double {mustBeNumeric}   % absolute coordinates
        dimension (1,1) double {mustBeNumeric}  % agent as a circle 
        attached (1,1) logical % is the agent attached to the load? 
        index_s;
        color;  % color of the agent in plots
        
        cargo RectangularCargo % object representing the load the agents have to move around
        map binaryOccupancyMap % map where the agent is working (needed for simulations)
        
        Ts (1,1) double {mustBeNumeric} % sampling time
        comm_range (1,1) double {mustBeNumeric} % max range to an agent that enables communication
        lidar_range (1,1) double {mustBeNumeric} % max distance at which an obstacle can be detected
        max_speed (1,1) double {mustBeNumeric} % [m/s] maximum speed reachable by the agent
        
        msg_in % recived message
        Neighbours % positions and names of in range agents
        Neighbours_scan  % scanned area of the Neighbours
        formation_VC Voronoi  % Voronoi cell used for formation
        obstacle_VC  Voronoi  % Voronoi cell used for obstacle avoidance
        
        bounds double {mustBeNumeric} % ideal distances to keep from each neighbour
        way_point (2,1) double {mustBeNumeric} % position to reach
        
        priority % measure of how much an agent wants to detach from the structure
    end
    
    methods
        function obj = agent(name, init_position, param, cargo, map)
            obj.name = name;
            obj.position = init_position;
            obj.cargo = cargo;
            obj.map = map;
            obj.dimension = param.radius;
            obj.comm_range = param.comm_range;
            obj.lidar_range = param.range;
            obj.max_speed = param.max_speed;
            
            obj.attached = false; % starts as unattached
            obj.msg_in = [];
            obj.Neighbours = [];
            obj.formation_VC = Voronoi(param.N_rho, param.N_phi, param.range);
            obj.obstacle_VC = Voronoi(param.N_rho, param.N_phi, param.range);
            obj.priority = 0;
            obj.color = rand(1,3);
        end
              
        % METHODS: communication
        
        function clearComms(obj)  
            % clear the comm buffer
            obj.msg_in = [];
        end
        
        
        function flag = isNeighbourInRange(obj, other, range)
            % check if the other agent is within a range
            q = obj.position - other.position;
            dist = sqrt(q' * q);
            flag = range > dist;
        end
        
        
        function sendMessage(obj, other, text) 
            % send a message to the other agent if it's within comm distance
            flag = obj.isNeighbourInRange(other, obj.comm_range);
            if(flag == true)
                other.msg_in = strcat(other.msg_in, text);
            end
        end
        
        
        function sendScan(obj, other, scan)
            % to ease the computation, the scanned area of an agent is sent
            % directily into a other agent buffer.
            flag = obj.isNeighbourInRange(other, obj.comm_range);
            
            if(flag == true)
                if(isempty(other.Neighbours_scan) == true)
                    other.Neighbours_scan(1).scan = scan;
                    other.index_s = 1;
                else
                    other.index_s = other.index_s + 1;
                    other.Neighbours_scan(other.index_s).scan = scan;
                end
            end
        end
        
        
        function decodeTextIn(obj)
            % takes the message that the agent recived and tries to decode
            % it
            if(isempty(obj.msg_in) == false)
                texts = split(obj.msg_in,';');
                for i = 1:length(texts)-1
                    cmds = split(texts(i),',');
                    for j = 1:length(cmds)
                        obj.executeCmd(cmds(j));
                    end
                end
            end
        end
        
        
        function clearNeighbours(obj)
            global index;
            index = 1;
            obj.Neighbours = [];
        end
        
        
        function executeCmd(obj, cmd)
            global index;
            % decode and execute the commands
            exe = cell2mat(cmd);
            if(isempty(exe) == false)
                if(exe(1) == 'N') % name of the agent
                    if(isempty(obj.Neighbours))
                        % init the structure
                        obj.Neighbours = struct('name', exe(2:end), 'position', [0,0], 'attached', false);
                        index = 1;
                    else
                        % check if the neighbours have been already met 
                        index = findStructFromName(obj.Neighbours, exe(2:end));
                        if(isnan(index) == true)
                            obj.Neighbours(end+1) = struct('name',exe(2:end), 'position', [0,0], 'attached', false);
                            index = length(obj.Neighbours);
                        end
                        % else the index is stored in order to compute the
                        % following commands
                    end
                elseif(exe(1) == 'X') % x position of the agent
                    obj.Neighbours(index).position(1) = str2double(exe(2:end));
                elseif(exe(1) == 'Y') % y position of the agent
                    obj.Neighbours(index).position(2) = str2double(exe(2:end));
                elseif(exe(1) == 'A') % agent is attached
                    obj.Neighbours(index).attached = true;
                elseif(exe(1) == 'D') % agent is attached
                    obj.Neighbours(index).attached = false;
                end
            end
        end
        
        % METHODS: actions
        
        function move(obj, input_velocity) 
            % compute the position at the next integration step.
            % Saturation of speed
            if(obj.max_speed > input_velocity)
                velocity = input_velocity;
            else
                velocity = obj.max_speed;
            end
            if(size(velocity, 2) ~= 1)
                obj.position = obj.position + obj.Ts * velocity';
            else
                obj.position = obj.position + obj.Ts * velocity;
            end
        end
        
        
        function moveToCentroid(obj, kp, centroid)
            % move the agent towards the previously the given centroid
            
            if(isempty(centroid) == false)
                velocity = kp * centroid; % centroid is in relative coordinates 
                move(obj, velocity);
            else
                error('Trying to move towards centroid but it has not been computed');
            end
        end
        
        
        function obj = attach(obj) 
            % if possible attach the robot to the load
            obj.attached = isInsideRectLoad(obj);
            if(obj.attached == false)
                warning('Attach operation outside the load area')
            end
        end
        
        
        function obj = detach(obj) 
            % detach the agent from the load
            obj.attached = false;
        end
        
        % METHODS: voronoi cell
        
        function s = scan(obj, obs)
            % simulate a scan of the nearby area using a lidar sensor. A
            % moving obstacle object can be passed to increase complexity.
            % moreover, other agents vision occlusion can be considered by
            % setting their positions with neighbours. Note that it the
            % last case, the agents are considered with the same radius
            
            s = zeros(obj.formation_VC.phi_n, 2);
            new_map = binaryOccupancyMap(obj.map); % copy
            tmp_map = binaryOccupancyMap(obj.map); % copy
            if(isempty(obs) == false)
                obs_map = addDisksToMap(obj.map, obs.center, obs.radius);
                syncWith(new_map, obs_map);
            end
            
            for n = 1:obj.formation_VC.phi_n
                % the current orientation of the rover isn't available in
                % the current version of the code, so it is set to 0°,
                % always facing right. 
                phi = n * obj.formation_VC.phi_res;
                pose = [obj.position', 0]; 
                point = rayIntersection(new_map, pose, phi, obj.lidar_range);
                
                if(isnan(point(1)) == true && isnan(point(2)) == true)
                    % free line of sight
                    s(n) = obj.lidar_range;
                else
                    % obstacle in line of sight
                    point_local = (point - obj.position');
                    s(n,1) = sqrt(point_local * point_local'); % save the distance
                end
                s(n,2) = phi;
            end
        end
        
        
        function computeVisibilitySet(obj, obs)
            % compute the visibility set of the agent. Has to be computed
            % before the Voronoi cell calculation
            agent_scan = obj.scan(obs);
            obj.obstacle_VC.visibilitySet(agent_scan);
            obj.formation_VC.visibilitySet(agent_scan);
        end
        
        
        function computeCellCollisionAvoidance(obj, type)
            % compute cell avoiding collisions with agents + voroni cell
            if(nargin < 2)
                type = 'All';
            end
            % select agents that satisfy type
            ids = getTypeIds(type, obj.Neighbours);
            tmp_NB = obj.Neighbours(ids);
                
            obj.formation_VC.computeCell(obj.position, tmp_NB, 2 * obj.dimension);
        end
        
        
        function computeCellFormation(obj, relax_factor, type)
            % uses collision avoidance setting as min distance the distance
            % between ideal position and each agent, considering a
            % relax_factor
            if(nargin < 3)
                type = 'All';
            end
            % select agents that satisfy type
            ids = getTypeIds(type, obj.Neighbours);
            tmp_NB = obj.Neighbours(ids);
            
            formation_lower_bound = obj.bounds - relax_factor;
            obj.formation_VC.computeCell(obj.position, tmp_NB, formation_lower_bound);
        end
        
        
        function applyVoronoiCargoLimits(obj, offset)
            % apply cargo limits on the cell tessellation
            obj.formation_VC.applyCargoLimits(obj.position, obj.cargo, offset);
        end
        
        
        function applyConnectivityMaintenance(obj, relax_factor, type)
            % compute the union between the neighbours visibility sets
            if(isempty(obj.Neighbours_scan) == true)
                error('agents did not share their scans!');
            end
            if(nargin < 3)
                type = 'All';
            end
            % select agents that satisfy type
            ids = getTypeIds(type, obj.Neighbours);
            
            if(nargin > 1 && isempty(relax_factor) == false) 
                upper_bounds = min(obj.lidar_range, obj.bounds + relax_factor);
            else
                upper_bounds = obj.lidar_range * ones(1,size(obj.Neighbours, 2));
            end
            % POSSIBLE FIX: iterate for every neighbour outside!!!
            % send only info of the selected neighbours
            obj.formation_VC.unionVisibilitySets(obj.position,...
                upper_bounds(ids), obj.Neighbours(ids), obj.Neighbours_scan(ids));
        end
        
        
        function applyLiberalConnectivityMaintenance(obj, relax_factor, type)
            % a spot has a greater value the more agents share it
            if(isempty(obj.Neighbours_scan) == true)
                error('agents did not share their scans!');
            end
            if(nargin < 3)
                type = 'All';
            end
            % select agents that satisfy type
            ids = getTypeIds(type, obj.Neighbours);
            
            if(nargin > 1 && isempty(relax_factor) == false) 
                upper_bounds = min(obj.lidar_range, obj.bounds + relax_factor);
            else
                upper_bounds = obj.lidar_range * ones(1,size(obj.Neighbours, 2));
            end
            % POSSIBLE FIX: iterate for every neighbour outside!!!
            % send only info of the selected neighbours
            obj.formation_VC.voteVisibilitySets(obj.position,...
                upper_bounds(ids), obj.Neighbours(ids), obj.Neighbours_scan(ids));
        end
        
        
        function clearScan(obj)
            % always to use after usage of sendScan so that the index gets reset       
            obj.index_s = 1;
            obj.Neighbours_scan = [];
        end
        
        % Density methods
        
        function applyVoronoiFarFromCargoDensity(obj, K)
            % should give an equivalent result to computeVoronoiCellCentroidAwayCenterMass
            % define density function
            ref = obj.position - (obj.cargo.center + obj.cargo.center_mass);
            fun_d = @(rho,phi) K * sqrt((rho * [cos(phi);sin(phi)] + ref)'...
                * (rho * [cos(phi);sin(phi)] + ref));
            % apply density function to the actual voronoi cell
            obj.formation_VC.applyDensity(fun_d);
        end
        
        
        function applyVoronoiPointDensity(obj, point, sf)
            % apply the density function of an exponential centerd in the
            % point position. The point is defined in the global refernce
            % frame.
            % NOTE: sf is the spread factor
            fun_dx = @(rho,phi) obj.position(1) + rho * cos(phi) - point(1);
            fun_dy = @(rho,phi) obj.position(2) + rho * sin(phi) - point(2);
            % define density function
            fun_dist = @(rho,phi) sqrt((fun_dx(rho,phi))^2 + (fun_dy(rho,phi))^2);
            % density exponential expression
            fun_d = @(rho,phi) exp(-fun_dist(rho,phi) / sf);
            % apply density
            obj.formation_VC.applyDensity(fun_d);
        end
        
        
        function applyVoronoiPointDensityLocal(obj, point, sf)
            % apply the density function of an exponential centerd in the
            % point position. The point is defined in the global refernce
            % frame and if not inside the circle of radius the lidar range,
            % the point is set on the boarder in the direction of the 
            % original one.
            % NOTE: sf is the spread factor
            distance = distance2D(point, obj.position);
            direction = (point - obj.position) / distance;
            if(distance > obj.lidar_range)
                % set the position of the point on the boarder
                point_updated = obj.position + direction * obj.lidar_range;
            else
                point_updated = point;
            end
            applyVoronoiPointDensity(obj, point_updated, sf);
        end
        
        
        function applyVoronoiWayPointDensity(obj, sf)
            % apply the density function of an exponential centerd in the
            % way point
            applyVoronoiPointDensity(obj, obj.way_point, sf);
        end
                
        % METHODS: path planning ------------------------------------------
        
        function way_point = computeWayPoint(obj, cargo_final_position)
            % compute the relative position that the agent should reach the
            % input pose has to be structured as [x; y; theta]
            delta_position = obj.position - obj.cargo.center;
            delta_angle = cargo_final_position(3) - obj.cargo.orientation; 
            way_point = cargo_final_position(1:2) + ...
                rotationMatrix(delta_angle) * delta_position;
            obj.way_point = way_point;
        end
        
        
        function c = getFormationCentroid(obj)
            % return the centroid relative to the formation cell
            c = obj.formation_VC.computeCentroid(); 
        end
        
        
        function c = getObstacleCentroid(obj)
            % return the centroid relative to the obstacle cell
            c = obj.obstacle_VC.computeCentroid(); 
        end
        
        % METHODS: auxiliary ----------------------------------------------   
        
        function r = isInsideRectLoad(obj) 
            % check weather the agent is within the designated area
            r = isInside(obj.cargo, obj.position, 0);
        end
        
        
        function d = getDistanceFromCOM(obj)
            % compute the distance form the center of mass of the cargo
            pos_diff = obj.position - obj.cargo.center - obj.cargo.center_mass;
            d = sqrt(pos_diff' * pos_diff);
        end
        
        
        function Neighbours_local = getNeighboursLocalPosition(obj)
            % return the Neighbours positions in local coordinates
            Neighbours_local = zeros([size(obj.Neighbours,1), 2]);
            
            for i = 1:length(obj.Neighbours)
                Neighbours_local(i,:) = ...
                    global2local(obj.position, obj.Neighbours(i).position'); 
            end
        end
        
        
        function reached = isWayPointReached(obj, error)
            % check if the position of the agent is within an error away
            % from the waypoint location.
            q = obj.way_point - obj.position;
            dist = sqrt(q' * q);
            reached = dist < error;
        end
        
        
        function fixBounds(obj)
            % set as bounds the distance to its neighbours
            n = length(obj.Neighbours);
            bds = zeros(n, 1);
            for i = 1:n
                bds(i) = distance2D(obj.Neighbours(i).position', obj.position);
            end
            obj.bounds = bds;
        end
        
        
        function hit = checkHit(obj, Obs)
            % check if the agent and obstacle collide
            c_dist = sqrt((obj.position - Obs.center)' * (obj.position - Obs.center));
            hit = c_dist - obj.dimension - Obs.radius < 0;
        end
        
        % PRIORITY METHODS ------------------------------------------------
        
        function [p, d] = computePriorityP(obj, Kp)
            % compute proportional priority wrt. an obstacle proximity
            c_o = obj.getObstacleCentroid();
            d = sqrt(c_o' * c_o); 
            p = d * Kp;
            obj.priority = p;
        end
        
        
        function [p,d] = computePriorityPD(obj, Kp, Kd, last_d)
            % compute proportional and derivative priority wrt. an obstacle 
            % proximity
            c_o = obj.getObstacleCentroid();
            d = sqrt(c_o' * c_o);
            p = d * Kp + (d - last_d) * Kd;
            obj.priority = p;
        end
        
        
        function [m_obs, m_for] = centroidsModule(obj)
            % compute the distance of both centroids
            c_o = obj.getObstacleCentroid();
            c_f = obj.getFormationCentroid();
            m_obs = sqrt(c_o' * c_o);
            m_for = sqrt(c_f' * c_f);
        end
        
        
        function [m] = centroidModule(obj)
            % compute the distance of the sum of both centroids
            c_o = obj.getObstacleCentroid();
            c_f = obj.getFormationCentroid();
            c   = c_o + c_f;
            m = sqrt(c' * c);
        end
        
        
        % METHODS: representation -----------------------------------------
        
        function plot(obj, plot_name_flag) 
            % represent the agent on a 2D plain in red if free to move, in 
            % blue if attached
            hold on
            offset = 0.1;
            if(obj.attached == true)
                plot(obj.position(1), obj.position(2), 'ob')
            else
                plot(obj.position(1), obj.position(2), 'or')
            end
            viscircles(obj.position', obj.dimension, 'color', obj.color);
            if obj.attached == true
                status = '(A)';
            else
                status = '(D)';
            end
            if(plot_name_flag == true)
                text(obj.position(1) + offset, obj.position(2), strcat(obj.name, status));
            end
            hold off
        end
        
        
        function plotVoronoiCellDetailed(obj, step, type)
            % plot a detailed version of the of the voronoi cell
            % considering the density of every point
            if(nargin < 3)
                obj.formation_VC.plot(obj.position, step);
            elseif(strcmp(type, 'Formation'))
                obj.formation_VC.plot(obj.position, step);
            elseif(strcmp(type, 'Obstacle'))
                obj.obstacle_VC.plot(obj.position, step);
            else
                error(strcat('Error: wrong type: ', type, 'of voronoi cell in plot'))
            end
        end
        
        
        function print(obj)
            % print basic data of the agent
            fprintf('Agent %s info:\n', obj.name);
            fprintf('position: %f, %f \t centroid position %f, %f \n', ...
                obj.position, obj.formation_VC.centroid);
            fprintf('way point %f, %f \n', obj.way_point);
        end
        
        
        function printStatus(obj)
            % print attached status of the agent
            fprintf(obj.name);
            if(obj.attached == true)
                s = 'YES';
            else
                s = 'NO';
            end
            fprintf(strcat('\t attached \t', s));
        end
        
        % SETTERS ---------------------------------------------------------
        function set.attached(obj, v)
            obj.attached = v;
        end
        function set.msg_in(obj, text)
            obj.msg_in = text;
        end
        function set.position(obj, pos)
            obj.position = pos;
        end
        function set.Ts(obj, t)
            obj.Ts = t;
        end
        function set.Neighbours(obj, info)
            obj.Neighbours = info;
        end
        function set.Neighbours_scan(obj, scan)
            obj.Neighbours_scan = scan;
        end
        function set.formation_VC(obj, v_cell)
            obj.formation_VC = v_cell;
        end
        function set.bounds(obj, pos)
            obj.bounds = pos; 
        end
    end
end

