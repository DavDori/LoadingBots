classdef agent < handle
    %AGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name
        position (2,1) double {mustBeNumeric}   % absolute coordinates
        dimension (1,1) double {mustBeNumeric}  % agent as a circle 
        attached (1,1) logical % is the agent attached to the load? 
        
        cargo rect_load % object representing the load the agents have to move around
        map binaryOccupancyMap % map where the agent is working (needed for simulations)
        
        Ts (1,1) double {mustBeNumeric} % sampling time
        comm_range (1,1) double {mustBeNumeric} % max range to an agent that enables communication
        lidar_range (1,1) double {mustBeNumeric} % max distance at which an obstacle can be detected
        
        msg_in
        Neighbours % positions and names of in range agents
        Voronoi_cell double {mustBeNumeric} % discretization of the space arount the agent
        centroid (1,2) double {mustBeNumeric} % relative direction of the centroid
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
            obj.Voronoi_cell = zeros(param.N_rho, param.N_phi); 
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
        
        
        function moveToCentroid(obj, kp)
            % move the agent towards the previously calculated centroid
            
            if(isempty(obj.centroid) == false)
                velocity = kp * obj.centroid; % centroid is in relative coordinates 
                move(obj, velocity);
            else
                error('Trying to move towards centroid but it has not been computed');
            end
        end
        
        function obj = attach(obj) % if possible attach the robot to the load
            obj.attached = isInsideRectLoad(obj);
            if(obj.attached == false)
                error('Attach operation outside the load area')
            end
        end
        
        function obj = detach(obj) % detach the agent from the load
            obj.attached = false;
        end
        
        % METHODS: voronoi cell
        
        function s = scan(obj)
            % scan the nearby area using a lidar sensor
            res_phi = 2 * pi / size(obj.Voronoi_cell, 2);
            s = zeros(size(obj.Voronoi_cell, 2),2);

            for n = 1:size(obj.Voronoi_cell, 2)
                % the current orientation of the rover isn't available in
                % the current version of the code, so it is set to 0°,
                % always facing right. 
                phi = n * res_phi;
                pose = [obj.position', 0]; 
                point = rayIntersection(obj.map, pose, phi, obj.lidar_range);
                
                if(isnan(point(1)) == true && isnan(point(2)) == true)
                    % no collisions
                    s(n) = obj.lidar_range;
                else
                    % consider the first point of collision
                    point_local = (point - obj.position');
                    s(n,1) = sqrt(point_local * point_local'); % save the distance
                end
                s(n,2) = phi;
            end
        end
        
        function computeVoronoiCell(obj)
            % discretize a circular space around the agent. Set to 1 the
            % point closer to the agent and to 0 those closer to the
            % neighbour agents.
            % define resolution of the angle and radius
            agent_scan = scan(obj);
            
            res_rho = obj.lidar_range / size(obj.Voronoi_cell, 1);
            res_phi = 2 * pi / size(obj.Voronoi_cell, 2);
            offset = 0.2; %[m] cargo offset so that the agent doesn't exeed the perimeter
                        
                        
            Neighbours_local_position = getNeighboursLocalPosition(obj);
            cell = zeros(size(obj.Voronoi_cell));
            
            % for every point in the cell check if it's closer to agent
            for i = 1:size(obj.Voronoi_cell, 2) % for every angle
                distance_index = ceil(agent_scan(i,1) / res_rho);
                for j = 1:distance_index % for some radius
                    rho = j * res_rho;
                    phi = i * res_phi;
                    [x, y] = polar2cartesian(rho,phi);
                    point = [x; y];
                    
                    if(isInside(obj.cargo, obj.position + point, offset))
                        cell(j,i) = ...
                        isCloser(Neighbours_local_position, point);
                    end
                end
            end
            obj.Voronoi_cell = cell;
        end
        
        function mass = computeVoronoiCellMass(obj, density_function)
            % approximation of every point as a trapezoid multiplied by the
            % density function
            res_rho = obj.lidar_range / size(obj.Voronoi_cell, 1);
            res_phi = 2 * pi / size(obj.Voronoi_cell, 2);
            mass = 0;
            for i = 1:size(obj.Voronoi_cell, 1)
                for j = 1:size(obj.Voronoi_cell, 2)
                    if(obj.Voronoi_cell(i,j) == 1)
                        rho = res_rho * i;
                        phi = j * res_phi;
                        s_base = 2 * (rho - res_rho / 2) * sin(res_phi / 2);
                        b_base = 2 * (rho + res_rho / 2) * sin(res_phi / 2);
                        mass = mass + density_function(rho, phi) * ... 
                            (s_base + b_base) * res_rho / 2;
                    end
                end
            end   
        end
        
        
        function c = computeVoronoiCellCentroid(obj, fun_m)
            % compute the centroid of the Voronoi cell considering a
            % mass function
           
            fun_x = @(rho,phi) rho * cos(phi) * fun_m(rho,phi);
            fun_y = @(rho,phi) rho * sin(phi) * fun_m(rho,phi);
            mass = computeVoronoiCellMass(obj, fun_m);
            
            x = computeVoronoiCellMass(obj, fun_x) / mass;
            y = computeVoronoiCellMass(obj, fun_y) / mass;
            c = [x;y];
            obj.centroid = c;
        end
        
        function c = computeVoronoiCellCentroidOpt(obj)
            %compute the voroni cell with a density function that increases
            %the more distant is the point to the center of mass of the
            %load
            ref = obj.position - (obj.cargo.center + obj.cargo.center_mass);
            fun_m = @(rho,phi) 10 * sqrt((rho * [cos(phi);sin(phi)] + ref)'...
                * (rho * [cos(phi);sin(phi)] + ref));
            c = computeVoronoiCellCentroid(obj, fun_m);
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
        
        function plot(obj) % represent the agent on a 2D plain in red if free to move, in blue if attached
            hold on
            if(obj.attached == true)
                plot(obj.position(1), obj.position(2), 'ob')
            else
                plot(obj.position(1), obj.position(2), 'or')
            end
            circle(obj.position(1), obj.position(2), obj.dimension);
            hold off
        end
        
        function plotVoronoiCell(obj)
            hold on
            res_rho = obj.lidar_range / size(obj.Voronoi_cell, 1);
            res_phi = 2 * pi / size(obj.Voronoi_cell, 2);
            
            for i = 1:size(obj.Voronoi_cell, 1) % rho
                for j = 1:size(obj.Voronoi_cell, 2) % phi
                    % represent every point of cell
                    if(obj.Voronoi_cell(i,j) == 1)
                        [x_local,y_local] = polar2cartesian(i * res_rho, j * res_phi);
                        local_p = [x_local, y_local]';
                        global_p = local2global(obj.position, local_p);
                        color = [1,1,1] * obj.Voronoi_cell(i,j) / max(obj.Voronoi_cell);
                        plot(global_p(1),global_p(2), strcat('o',color));
                    end
                end
            end
            hold off
        end
        
        function plotVoronoiCellFast(obj, color)
            % consider only the outter perimeter of the cell
            res_rho = obj.lidar_range / size(obj.Voronoi_cell, 1);
            res_phi = 2 * pi / size(obj.Voronoi_cell, 2);
            edges = zeros(size(obj.Voronoi_cell, 2), 2); 
            
            for i = 1:size(obj.Voronoi_cell, 2) % rho
                [x_local,y_local] = polar2cartesian(...
                    sum(obj.Voronoi_cell(:,i)) * res_rho, i * res_phi);
                local_p = [x_local, y_local]';
                edges(i,:) = local2global(obj.position, local_p);
            end
            plot(edges(:,1), edges(:,2), 'Color', color);
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
        function set.centroid(obj, c)
            obj.centroid = c;
        end
    end
end

