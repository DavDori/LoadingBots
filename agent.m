classdef agent < handle
    %AGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name
        position (2,1) double {mustBeNumeric}   % absolute coordinates
        dimension (1,1) double {mustBeNumeric}  % agent as a circle 
        attached (1,1) logical % is the agent attached to the load? 
        load_box rect_load % object representing the load the agents have to move around
        
        Ts (1,1) double {mustBeNumeric} % sampling time
        comm_range (1,1) double {mustBeNumeric} % max range to an agent that enables communication
        lidar_range (1,1) double {mustBeNumeric} % max distance at which an obstacle can be detected
        
        msg_in
        Neighbours % positions and names of in range agents
        Voronoi_cell double {mustBeNumeric} % discretization of the space arount the agent
    end
    
    methods
        function obj = agent(name, init_position, param, box)
            obj.name = name;
            obj.position = init_position;
            obj.load_box = box;
            obj.dimension = param.radius;
            obj.comm_range = param.comm_range;
            obj.lidar_range = param.range;
            obj.attached = false; % starts as unattached
            obj.msg_in = [];
            obj.Neighbours = [];
            obj.Voronoi_cell = zeros(param.N_rho, param.N_phi); 
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
        
        % METHODS: communication
        
        function clearComms(obj)  % clear the comm buffer
            obj.msg_in = [];
        end
        
        function sendMessage(obj, other, text) % send a message to the other agent if it's within comm distance
            q = obj.position - other.position;
            dist = sqrt(q' * q);
            if(obj.comm_range > dist)
                other.msg_in = strcat(other.msg_in, text);
            end
        end
        
        function decodeTextIn(obj)
            % takes the message that the agent recived and tries to decode
            % it
            texts = split(obj.msg_in,';');
            for i = 1:length(texts)
                cmds = split(texts(i),',');
                for j = 1:length(cmds)
                    obj.executeCmd(cmds(j));
                end
            end
        end
        
        function executeCmd(obj, cmd)
            exe = cell2mat(cmd);
            if(isempty(exe) == false)
                if(exe(1) == 'N') % name of the agent
                    if(isempty(obj.Neighbours))
                        obj.Neighbours = struct('name',exe(2:end), 'position', [0,0]);
                    else
                        obj.Neighbours(end+1) = struct('name',exe(2:end), 'position', [0,0]);
                    end
                elseif(exe(1) == 'X') % x position of the agent
                    obj.Neighbours(end).position(1) = str2double(exe(2:end));
                elseif(exe(1) == 'Y') % y position of the agent
                    obj.Neighbours(end).position(2) = str2double(exe(2:end));
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
        
        function obj = attach(obj) % if possible attach the robot to the load
            obj.attached = isInsideRectLoad(obj);
            if(obj.attached == false)
                error('Attach operation outside the load area')
            end
        end
        
        function obj = detach(obj) % detach the agent from the load
            obj.attached = false;
        end
        
        % METHODS: control
        
        function computeVoronoiCell(obj)
            % discretize a circular space around the agent. Set to 1 the
            % point closer to the agent and to 0 those closer to the
            % neighbour agents.
            % define resolution of the angle and radius
            res_rho = obj.lidar_range / size(obj.Voronoi_cell, 1);
            res_phi = 2 * pi / size(obj.Voronoi_cell, 2);
            
            Neighbours_local_position = getNeighboursLocalPosition(obj);
            cell = zeros(size(obj.Voronoi_cell));
            
            % for every point in the cell check if it's closer to agent
            for i = 1:size(obj.Voronoi_cell, 1)
                for j = 1:size(obj.Voronoi_cell, 2)
                    rho = i * res_rho;
                    phi = j * res_phi;
                    [x, y] = polar2cartesian(rho,phi);
                    point = [x, y];
                    cell(i,j) = ...
                        isCloser(Neighbours_local_position, point);
                end
            end
            obj.Voronoi_cell = cell;
        end
        
        
        function r = isInsideRectLoad(obj) 
            % check weather the agent is within the designated area
            r = isInside(obj.load_box, obj.position);
        end
        
        % METHODS: auxiliary
        
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
            hold off
        end
        
        function plotVoronoiCell(obj, color)
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
            plot(edges(:,1), edges(:,2), color);
        end
    end
end

