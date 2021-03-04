classdef tester
    %TESTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        map
        param
        Ts
        max_error
    end
    
    methods
        function obj = tester(map, parameters, sampling_time, position_error)
            %TESTER Construct an instance of this class
            obj.map = map;
            obj.param = parameters;
            obj.Ts = sampling_time;
            obj.max_error = position_error;
        end
        
        
        function flag = centroidOmogeneus(obj)
            % Test wheather the centroid of an agent without any obstacle
            % around is the agent position
            center = [2 ; 2]; % [m]
            center_mass = [0;0];        % [m]
            dimensions = [1.5; 1];      % [m]
            orientation = pi/2;         % [rad]
            cargo = rect_load(center, center_mass, orientation, dimensions);
            
            agents(1) = agent('001', [2; 2.5], obj.param, cargo, obj.map);
            robot_flock = flock(agents, cargo, obj.Ts);
            
            robot_flock.computeVoronoiTessellation();
            c = robot_flock.computeVoronoiCentroids();
            e = abs(c' - robot_flock.agents.position());
            if(e(1) < obj.max_error && e(2) < obj.max_error)
                flag = false;
            else
                flag = true; % test passed
            end 
        end
        
        
        function flag = moveToCentroid(obj)
            % test the movement of the robot. Given a the position of the
            % centroid and set the proportional parameter such that the
            % robot reaches exactly the centroid after a simulation step.
            center = [2 ; 2]; % [m]
            center_mass = [0;0];        % [m]
            dimensions = [1.5; 1];      % [m]
            orientation = pi/2;         % [rad]
            cargo = rect_load(center, center_mass, orientation, dimensions);

            agents(1) = agent('001', cargo.center, obj.param, cargo, obj.map);
            
            robot_flock = flock(agents, cargo, obj.Ts);
            centroid = [0.2; 0.2]; % custom centroid position
            % set centroid 
            robot_flock.agents.Voronoi_cell.centroid = centroid;
            % calculate the speed needed to reach the centroid in one step
            kd = 1 / obj.Ts;
            
            robot_flock.moveToCentroids(kd);
            
            check_p = abs(robot_flock.agents.position - center - centroid) < [obj.max_error; obj.max_error];
            
            flag = check_p(1) == true && check_p(2) == true;
            % true -> test passed
            % false -> test failed
        end
        
        
        function flag = collisionDetection(obj, flag_plot)
            % Test the collision detection with two agents, their centroids
            % should move in opposite directions and their Voronoi cells
            % should show a gap instead of touching
            
            center = [2 ; 2]; % [m]
            center_mass = [0;0];        % [m]
            dimensions = [1.5; 1];      % [m]
            orientation = pi/2;         % [rad]
            cargo = rect_load(center, center_mass, orientation, dimensions);
            
            dist = obj.param.radius * 1.1;
            agents(1) = agent('001', [2 + dist; 0], obj.param, cargo, obj.map);
            agents(2) = agent('002', [2 - dist; 0], obj.param, cargo, obj.map);
            
            robot_flock = flock(agents, cargo, obj.Ts);
            robot_flock.meetNeighbours();
            robot_flock.computeVoronoiTessellation();
            robot_flock.applyConstantDensity();
            c = robot_flock.computeVoronoiCentroids();
            % agent 1 should be going on the right
            v_1 = c(1,:)';
            check_1 = abs(v_1(2)) < obj.max_error && v_1(1) > 0;
            % agent 2 should be going on the left
            v_2 = c(2,:)';
            check_2 = abs(v_2(2)) < obj.max_error && v_2(1) < 0;
            
            if(flag_plot == true)
                figure()
                grid on
                hold on
                axis equal
                robot_flock.plotVoronoiTessellation();
                robot_flock.plot()
                robot_flock.plotCentroids();
                hold off
            end
            
            flag = check_1 == true && check_2 == true;
            % true -> test passed
            % false -> test failed
        end
        
        
        function flag = cargoLimits(obj)
            % Set the agent at one corner of the cargo and calculate the
            % Voronoi cell considering a 0 offset. The expected result
            % should be that only 1/4 of the cell is set to 1.
            center = [3; 3]; % [m]
            center_mass = [0;0];        % [m]
            dimensions = [1; 1];      % [m]
            orientation = 0;         % [rad]
            offset = 0;
            
            % cargo has to be a rectangle
            cargo = rect_load(center, center_mass, orientation, dimensions);
            vertex = computeVertexPositions(cargo); 
            % set the agent at 1 vertex
            agents(1) = agent('001', vertex(1,:), obj.param, cargo, obj.map);
            robot_flock = flock(agents, cargo, obj.Ts);
            robot_flock.meetNeighbours();
            robot_flock.computeVoronoiTessellationCargo(offset);
            % get the points in the cell that are set to 1
            p = sum(robot_flock.agents.Voronoi_cell.cell, 'all');
            % get the total amount of points
            p_tot = robot_flock.agents.Voronoi_cell.rho_n * robot_flock.agents.Voronoi_cell.phi_n;
            
            flag = (p / p_tot < 1/4 + obj.max_error) && (p / p_tot < 1/4 - obj.max_error);
            % true -> test passed
            % false -> test failed
        end
    end
end

