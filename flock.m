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
        
        function computeVoronoiTessellation(obj)
            % compute the Voronoi tessellation of a discretization of the
            % nearby area for every agent in the flock
            for a = obj.agents
                a.computeVoronoiCell(); 
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
        
        function moveToCentroids(obj)
            % move all the agents in direction of their centroids for a
            % sampling time.
            kp = 4;
            for a = obj.agents
                a.moveToCentroid(kp); 
            end
        end
        
        function spreadUnderCargo(obj, steps)
            % distributed maximum coverage application
            % update the position of the neighbours

            for i = 1:steps
                obj.meetNeighbours(); 
                obj.computeVoronoiTessellation();
                obj.computeVoronoiCentroidsOpt();
                obj.moveToCentroids();
            end
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
            % plot the Voronoi cells of every robot
            hold on
            for a = obj.agents
                a.plotVoronoiCell(); 
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
    end
end

