classdef agent < handle
    %AGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name % id of the agent
        position (2,1) double {mustBeNumeric}   % absolute coordinates
        dimension (1,1) double {mustBeNumeric}  % agent as a circle 
        attached (1,1) logical % is the agent attached to the load? 
        
        cargo rect_load % object representing the load the agents have to move around
        map binaryOccupancyMap % map where the agent is working (needed for simulations)
        
        Ts (1,1) double {mustBeNumeric} % sampling time
        comm_range (1,1) double {mustBeNumeric} % max range to an agent that enables communication
        lidar_range (1,1) double {mustBeNumeric} % max distance at which an obstacle can be detected
        
        msg_in % recived message
        Neighbours % positions and names of in range agents
        Voronoi_cell Voronoi
        
        ideal_position (2,1) double {mustBeNumeric} % ideal position relative to the cargo reference frame
        way_point (2,1) double {mustBeNumeric} % position to reach
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
            obj.attached = false; % starts as unattached
            obj.msg_in = [];
            obj.Neighbours = [];
            obj.Voronoi_cell = Voronoi(param.N_rho, param.N_phi, param.range);
        end
              
        % METHODS: communication
        
        function clearComms(obj)  % clear the comm buffer
            obj.msg_in = [];
        end
        
        
        function sendMessage(obj, other, text) 
            % send a message to the other agent if it's within comm distance
            q = obj.position - other.position;
            dist = sqrt(q' * q);
            if(obj.comm_range > dist)
                other.msg_in = strcat(other.msg_in, text);
            end
        end
        
        
        function decodeTextIn(obj)
            % takes the message that the agent recived and tries to decode
            % it
            if(isempty(obj.msg_in) == false)
                texts = split(obj.msg_in,';');
                for i = 1:length(texts)
                    cmds = split(texts(i),',');
                    for j = 1:length(cmds)
                        obj.executeCmd(cmds(j));
                    end
                end
            end
        end
        
        
        function clearNeighbours(obj)
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
                        obj.Neighbours = struct('name', exe(2:end), 'position', [0,0]);
                        index = 1;
                    else
                        % check if the neighbours have been already met 
                        index = findStructFromName(obj.Neighbours, exe(2:end));
                        if(isnan(index) == true)
                            obj.Neighbours(end+1) = struct('name',exe(2:end), 'position', [0,0]);
                            index = length(obj.Neighbours);
                        end
                        % else the index is stored in order to compute the
                        % following commands
                    end
                elseif(exe(1) == 'X') % x position of the agent
                    obj.Neighbours(index).position(1) = str2double(exe(2:end));
                elseif(exe(1) == 'Y') % y position of the agent
                    obj.Neighbours(index).position(2) = str2double(exe(2:end));
                end
            end
        end
        
        % METHODS: actions
        
        function move(obj, velocity) % compute the position at the next integration step               
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
                error('Attach operation outside the load area')
            end
        end
        
        
        function obj = detach(obj) 
            % detach the agent from the load
            obj.attached = false;
        end
        
        % METHODS: voronoi cell
              
        
        function s = scan(obj)
            % simulate a scan of the nearby area using a lidar sensor
            s = zeros(obj.Voronoi_cell.phi_n, 2);

            for n = 1:obj.Voronoi_cell.phi_n
                % the current orientation of the rover isn't available in
                % the current version of the code, so it is set to 0°,
                % always facing right. 
                phi = n * obj.Voronoi_cell.phi_res;
                pose = [obj.position', 0]; 
                point = rayIntersection(obj.map, pose, phi, obj.lidar_range);
                
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
        
        
        function computeVoronoiCell(obj)
            % compute the Voronoi Cell of the agent
            agent_scan = scan(obj);
            Neighbours_local_position = getNeighboursLocalPosition(obj);
            obj.Voronoi_cell.computeCell(agent_scan ,Neighbours_local_position, obj.dimension);
        end
                 
        
        function applyVoronoiFarFromCargoDensity(obj)
            % should give an equivalent result to computeVoronoiCellCentroidAwayCenterMass
            % define density function
            ref = obj.position - (obj.cargo.center + obj.cargo.center_mass);
            gain = 10;
            fun_d = @(rho,phi) gain * sqrt((rho * [cos(phi);sin(phi)] + ref)'...
                * (rho * [cos(phi);sin(phi)] + ref));
            % apply density function to the actual voronoi cell
            obj.Voronoi_cell.applyDensity(fun_d);
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
            obj.Voronoi_cell.applyDensity(fun_d);
        end
        
        
        function applyVoronoiWayPointDensity(obj, sf)
            % apply the density function of an exponential centerd in the
            % way point
            applyVoronoiPointDensity(obj, obj.way_point, sf);
        end
        
        
        function applyVoronoiIdealPositionDensity(obj, sf)
            % apply the density function of an exponential centered in the
            % ideal position
            % the ideal_position is defined in the cargo refernce frame but
            % it has to be changed in the global ref frame
            point = rotationMatrix(obj.cargo.orientation)' * obj.ideal_position(1:2) + obj.cargo.center;
            applyVoronoiPointDensity(obj, point, sf);
        end
        
        
        function applyVoronoiCargoLimits(obj, offset)
            obj.Voronoi_cell.applyCargoLimits(obj.position, obj.cargo, offset);
        end
        
                
        % METHODS: path planning
        
        function way_point = computeWayPoint(obj, cargo_final_position)
            % compute the relative position that the agent should reach the
            % input pose has to be structured as [x; y; theta]
            delta_position = obj.position - obj.cargo.center;
            delta_angle = cargo_final_position(3) - obj.cargo.orientation; 
            way_point = cargo_final_position(1:2) + rotationMatrix(delta_angle) * delta_position;
            obj.way_point = way_point;
        end
        
        % METHODS: auxiliary    
        
        function r = isInsideRectLoad(obj) 
            % check weather the agent is within the designated area
            r = isInside(obj.cargo, obj.position, 0);
        end
        
        
        function Neighbour_local = getNeighboursLocalPosition(obj)
            % return the Neighbours positions in local coordinates
            Neighbour_local = zeros([size(obj.Neighbours,1), 2]);
            
            for i = 1:length(obj.Neighbours)
                Neighbour_local(i,:) = ...
                    global2local(obj.position, obj.Neighbours(i).position'); 
            end
        end
        
        % METHODS: representation
        
        function plot(obj) 
            % represent the agent on a 2D plain in red if free to move, in 
            % blue if attached
            hold on
            if(obj.attached == true)
                plot(obj.position(1), obj.position(2), 'ob')
            else
                plot(obj.position(1), obj.position(2), 'or')
            end
            circle(obj.position(1), obj.position(2), obj.dimension);
            hold off
        end
        
        
        function print(obj)
            fprintf('Agent %s info:\n', obj.name);
            fprintf('position: %f, %f \t centroid position %f, %f \n', obj.position, obj.Voronoi_cell.centroid);
            fprintf('ideal position: %f, %f \t way point %f, %f \n', obj.ideal_position, obj.way_point);
        end
        
        
        % SETTERS
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
        function set.Voronoi_cell(obj, v_cell)
            obj.Voronoi_cell = v_cell;
        end
        function set.ideal_position(obj, pos)
            obj.ideal_position = pos; 
        end
    end
end

