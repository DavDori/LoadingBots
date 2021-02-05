classdef flock < handle
    %FLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        agents agent
        my_load rect_load
        n_agents (1,1) double {mustBeNumeric}
    end
    
    methods
        function obj = flock(agents_array, box, sampling_time)
            %FLOCK 
            obj.agents = agents_array;
            obj.n_agents = length(agents_array);
            obj.my_load = box;
            for a = obj.agents
                a.Ts = sampling_time;
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
            n = 1;
            for i = 1:nodes
                if(obj.agents(i).attached == true) % considers only attached agents
                    points(n,:) = obj.agents(i).position';
                    n = n + 1;
                end
            end
            flag = obj.my_load.isBalanced(points);
        end
        
        function meetNeighbours(obj)
            for i = 1:obj.n_agents
                for j = 1:obj.n_agents
                    if(i ~= j)
                        mex = strcat('N', obj.agents(i).name, ...
                                     ',X', num2str(obj.agents(i).position(1)),...
                                     ',Y', num2str(obj.agents(i).position(2)), ';');
                        obj.agents(i).sendMessage(obj.agents(j), mex);
                    end
                end
            end
            for agent = obj.agents
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
        
        function centroids = computeVoronoiCentroids(obj)
            % compute the centroid of every agent voronoi cell
            centroids = zeros(obj.n_agents, 2);
            for i = 1:obj.n_agents
                centroids(i,1:2) = obj.agents(i).computeVoronoiCellCentroid(); 
            end
        end
        
        % METHODS: representation
        
        function plot(obj)
            % representation in a 2D plane of the agents and the load
            hold on
            axis equal
            obj.my_load.plot('r')
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

