classdef flock < handle
    %FLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        agents agent
        cargo rect_load
        n_agents (1,1) double {mustBeNumeric}
    end
    
    methods
        function obj = flock(agents_array, box, sampling_time)
            %FLOCK 
            obj.agents = agents_array;
            obj.n_agents = length(agents_array);
            obj.cargo = box;
            for a = obj.agents
                a.Ts = sampling_time;
            end
        end
        
        
        function attachAll(obj)
            % attach all the agents on the board
            for a = obj.agents
                a.attach(); 
            end
        end
        
        
        function moveAgent(obj, index, velocity)
            % move a specific agent
            if(index < obj.n_agents)
                obj.agents(index).move(velocity);
            end
        end
        
        
        function flag = checkBalance(obj)
            % check if the load should be falling
            points = zeros([obj.n_agents, 2]);
            n = 0;
            for i = 1:obj.n_agents
                if(obj.agents(i).attached == true) % considers only attached agents
                    n = n + 1;
                    points(n,:) = obj.agents(i).position';
                end
            end
            flag = obj.cargo.isBalanced(points(1:n,:));
        end
        
        
        function sendNamePosition(obj)
            % every agent send a message to its neighbours containing name
            % and its absolute coordinates
            for i = 1:obj.n_agents
                for j = 1:obj.n_agents
                    if(i ~= j)
                        % string to send
                        mex = strcat('N', obj.agents(i).name, ...
                                     ',X', num2str(obj.agents(i).position(1)),...
                                     ',Y', num2str(obj.agents(i).position(2)), ';');
                        obj.agents(i).sendMessage(obj.agents(j), mex);
                    end
                end
            end
        end
        
        
        function meetNeighbours(obj)
            % send message to each agent and decode it so that every agent
            % will know its neighbours position.
            sendNamePosition(obj);
            for agent = obj.agents
                agent.clearNeighbours();
                agent.decodeTextIn();
                agent.clearComms();
            end
        end
        
        
        function computeVoronoiTessellationCargo(obj, offset)
            % compute the Voronoi tessellation of a discretization of the
            % nearby area for every agent in the flock considering the
            % perimeter of the cargo
            for a = obj.agents
                computeVoronoiCell(a, offset); 
            end
        end
        
        
        function computeVoronoiTessellation(obj)
            % compute the Voronoi tessellation of a discretization of the
            % nearby area for every agent in the flock 
            for a = obj.agents
                computeVoronoiCell(a); 
            end
        end
        
        
        function centroids = computeVoronoiCentroids(obj, fun_m)
            % compute the centroid of every agent voronoi cell
            centroids = zeros(obj.n_agents, 2);
            if(nargin == 1)
                fun_m = @(rho,phi) 1;
            end
            for i = 1:obj.n_agents
                centroids(i,1:2) = obj.agents(i).computeVoronoiCellCentroid(fun_m);
            end
        end
        
        
        function centroids = computeVoronoiCentroidsOpt(obj)
            % compute the centroid of every agent voronoi cell
            centroids = zeros(obj.n_agents, 2);

            for i = 1:obj.n_agents
                centroids(i,1:2) = obj.agents(i).computeVoronoiCellCentroidOpt();
            end
        end
        
        function setWayPoints(obj, dest)
            % given the destination of the cargo, compute the waypoints of
            % each agent
            for i = 1:obj.n_agents
                % calculate the destination for each agent
                dest_i = obj.agents(i).computeWayPoint(dest');
                fprintf('position of %s = [%f, %f]\n', obj.agents(i).name, dest_i);
            end
        end
        
        
        function centroids = computeVoronoiCentroidsNav(obj)
            % compute the voronoi centroids for each agent considering a
            % waypoint/destiantion to reach. Before this operation, the
            % waypoints should have been set.
            centroids = zeros(obj.n_agents, 2);
            sf = 1;
            for i = 1:obj.n_agents
                % calculate the destination for each agent
                centroids(i,1:2) = obj.agents(i).computeVoronoiCellCentroidMovement(sf);
            end
        end
        
        
        function moveToCentroids(obj, kp)
            % move all the agents in direction of their centroids for a
            % sampling time.
            for a = obj.agents
                centroid = a.centroid;
                moveToCentroid(a, kp, centroid); 
            end
        end
        
        
        function saveFormation(obj)
            % save the robots positions relative to the cargo reference 
            % frame, these positions should be the go to for every robot
            % when performing the attach operation.
            % the formation is saved with the reference frame of the cargo
            for a = obj.agents
                a.ideal_position = rotationMatrix(a.cargo.orientation) * (a.position - a.cargo.center);
            end
        end
        
        
        function spreadUnderCargo(obj, steps)
            % distributed maximum coverage application
            % update the position of the neighbours
            offset = 0.3;
            for i = 1:steps
                obj.meetNeighbours(); 
                obj.computeVoronoiTessellationCargo(offset);
                obj.computeVoronoiCentroids(); %opt or not
                obj.moveToCentroids(4);
            end
            % save current positions of the agents as reference for the
            % formation shape
            obj.saveFormation();
        end
            
        
        function waypoints = setTrajectory(obj, cmds)
            % calculate the waypoints given a set of commands. The motion
            % is modeled as a unicycle and the starting position is the
            % current position of the cargo center. The output is a
            % sequence of n points
            % Note: cmds structure: [v, w] and time is considered as 1 unit
            n = size(cmds,1); % rows
            waypoints = zeros(n, 3); % [x,y,theta]
            prev = [obj.agents(1).cargo.center; obj.agents(1).cargo.orientation]'; % initial pose
            for i = 1:n
                waypoints(i,:) = prev + unicycleModel(cmds(i,:), prev)';
                prev = waypoints(i,:);
            end
        end
                
        % METHODS: representation
        
        function plot(obj)
            % representation in a 2D plane of the agents and the load
            hold on
            axis equal
            obj.cargo.plot('r')
            for a = obj.agents
                a.plot();
            end
            hold off
        end
        
        
        function plotVoronoiTessellation(obj)
            % plot the limit perimeter of the Voronoi cells of every robot
            hold on
            for a = obj.agents
                a.plotVoronoiCellFast([rand,rand,rand]); 
            end
            hold off
        end
        
        
        function plotVoronoiTessellationDetailed(obj)
            % plot the Voronoi cells of every robot considering every point
            % of the cell
            hold on
            for a = obj.agents
                spread_factor = 1;
                dest = a.position - 0.5; % for testing
                fun_dist = @(rho,phi) sqrt((a.position(1) + rho * cos(phi) - dest(1))^2 + ...
                                       (a.position(2) + rho * sin(phi) - dest(2))^2);
                % density exponential expression
                fun_d = @(rho,phi) exp(-fun_dist(rho,phi) / spread_factor);
                
                a.plotVoronoiCell(fun_d); 
            end
            hold off
        end
        
        function plotCentroids(obj)
            hold on
            for a = obj.agents
                c = a.position' + a.centroid;
                plot([a.position(1), c(1)],[a.position(2), c(2)],'r');
                plot(c(1),c(2), 'xb');
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
    end
end

