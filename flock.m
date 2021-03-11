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
        
        
        function sendScan(obj)
            % every agent send a message to its neighbours containing name
            % and its absolute coordinates
            for i = 1:obj.n_agents
                scan = obj.agents(i).scan();
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
                a.computeVoronoiCell();
                a.applyVoronoiCargoLimits(offset);
            end
        end
        
        
        function computeVoronoiTessellation(obj)
            % compute the Voronoi tessellation of a discretization of the
            % nearby area for every agent in the flock 
            for a = obj.agents
                computeVoronoiCell(a); 
            end
        end
        
        
        function connectivityMaintenance(obj)
            % modify the voronoi cells of every agent to allow connectivity
            % maintenance
            for a = obj.agents
                a.applyConnectivityMaintenance();
            end
        end
        
        
        function centroids = computeVoronoiCentroids(obj)
            % compute the centroid of every agent voronoi cell
            centroids = zeros(obj.n_agents, 2);
            for i = 1:obj.n_agents
                centroids(i,1:2) = obj.agents(i).Voronoi_cell.computeCentroid();
            end
        end
        
        
        function applyFarFromCenterMassDensity(obj)
            % for every robot apply the far from center of mass density on
            % the Voronoi cell
            for a = obj.agents
                a.applyVoronoiFarFromCargoDensity();
            end
        end
        
        
        function applyWayPointDensity(obj, sf)
            % for every robot apply an exponential density function
            % centered on its way point on the voronoi cell. 'sf' is the
            % spread factor of the points
            for a = obj.agents
                a.applyVoronoiWayPointDensity(sf);
            end
        end
        
        
        function applyIdealPointDensity(obj, sf)
            % for every robot apply an exponential density function
            % centered on its way point on the voronoi cell. 'sf' is the
            % spread factor of the points
            for a = obj.agents
                a.applyVoronoiIdealPositionDensity(sf);
            end
        end
        
        
        function applySinglePointDensity(obj, point, sf)
            % for every robot apply an exponential density function
            % centered on a common point on the voronoi cell. 'sf' is the
            % spread factor of the point
            for a = obj.agents
                a.applyVoronoiPointDensity(point, sf);
            end
        end
        
        
        function applyConstantDensity(obj)
            % for every robot apply a constant density at its Voronoi cell
            % density
            for a = obj.agents
                a.Voronoi_cell.addConstantDensity();
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
        
        
        function moveToCentroids(obj, kp)
            % move all the agents in direction of their centroids for a
            % sampling time.
            for a = obj.agents
                centroid = a.Voronoi_cell.centroid;
                a.moveToCentroid(kp, centroid); 
            end
        end
                
        
        function spreadUnderCargo(obj, steps, offset)
            % distributed maximum coverage application
            % update the position of the neighbours
            kd = 4;
            for i = 1:steps
                obj.meetNeighbours(); 
                obj.computeVoronoiTessellationCargo(offset);
                obj.applyFarFromCenterMassDensity();
                obj.computeVoronoiCentroids();
                obj.moveToCentroids(kd);
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
        
        
        function fixFormation(obj)
            % should be computed after the flock has spread under the
            % cargo. It saves the current positions of the agents in their
            % ideal_position variable which is saved in the reference frame
            % of the cargo
            for a = obj.agents
                if(a.attached == false)
                    % if the agent is not attached, a warning is thrown
                    fprintf('WARNING: %s is not attached \n', a.name); 
                end
                d_pos = a.position - a.cargo.center;
                a.ideal_position = rotationMatrix(a.cargo.orientation) * d_pos;
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
                a.Voronoi_cell.plotFast(a.position, [rand,rand,rand]); 
            end
            hold off
        end
        
        
        function plotVoronoiTessellationDetailed(obj, step)
            % plot the Voronoi cells of every robot considering every point
            % of the cell
            hold on
            for a = obj.agents
                a.plotVoronoiCellDetailed(step); 
            end
            hold off
        end
        
        
        function plotCentroids(obj)
            hold on
            for a = obj.agents
                c = a.position + a.Voronoi_cell.centroid;
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
        
        
        function print(obj)
            % print basic info on the agents
            for a = obj.agents
                a.print();
                fprintf('\n');
            end
        end
    end
end

