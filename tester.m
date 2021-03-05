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
        
        
        function [flag_1, flag_2] = PointDensity(obj)
            % first test: point density inside the robot area, max density
            % should be on the point itsself;
            
            % second test: point density outside, max density sould be at
            % maximum rho, in the direction of the point.
            
            center = [2 ; 2]; % [m]
            center_mass = [0;0];        % [m]
            dimensions = [1.5; 1];      % [m]
            orientation = pi/2;         % [rad]
            cargo = rect_load(center, center_mass, orientation, dimensions);

            agents(1) = agent('001', cargo.center, obj.param, cargo, obj.map);
            
            robot_flock = flock(agents, cargo, obj.Ts);
            sf = 1;
            % first test
            point = center + obj.param.range / 2;
            
            robot_flock.computeVoronoiTessellation();
            robot_flock.applySinglePointDensity(point, sf);
            cell = robot_flock.agents(1).Voronoi_cell.cell_density;
            [~, index] = max(cell(:));
            [i, j] = ind2sub(size(cell), index);
            
            rho = i * robot_flock.agents(1).Voronoi_cell.rho_res;
            phi = j * robot_flock.agents(1).Voronoi_cell.phi_res;
            [dx, dy] = polar2cartesian(rho, phi);
            p = robot_flock.agents(1).position + [dx; dy];
            err_p = 10 * obj.max_error; % depends on the resolution of the cell
            flag_1 = abs(p(1) - point(1)) < err_p &&...
                        abs(p(2) - point(2)) < err_p;
                    
            % SECOND TEST
            point_2 = center + obj.param.range * 1.5;
            
            robot_flock.computeVoronoiTessellation();
            robot_flock.applySinglePointDensity(point_2, sf);
            cell = robot_flock.agents(1).Voronoi_cell.cell_density;
            [~, index] = max(cell(:));
            [i, j] = ind2sub(size(cell), index);
            
            rho = i * robot_flock.agents(1).Voronoi_cell.rho_res;
            phi = j * robot_flock.agents(1).Voronoi_cell.phi_res;
            
            phi_point = atan2(point_2(2) - center(2), point_2(1) - center(1));
            check_rho = rho >= obj.param.range - obj.max_error;
            % NOTE: atan2 returns a value between -pi and pi, on the other
            % hand, the values are regarded as from 0 to 2pi
            check_phi = abs(phi - phi_point) < obj.max_error * 10;
            flag_2 = check_rho && check_phi;
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
            
            flag = (abs(p / p_tot - 1/4) < obj.max_error);
            % true -> test passed
            % false -> test failed
        end
        
        
        function runAll(obj)
            e = obj.centroidOmogeneus();
            if(e == false)
                fprintf('centroid test: \t failed\n');
            else
                fprintf('centroid test: \t successeful\n');
            end
            e = obj.collisionDetection(false);
            if(e == false)
                fprintf('collision test: \t failed\n');
            else
                fprintf('collision test: \t successeful\n');
            end
            e = obj.cargoLimits();
            if(e == false)
                fprintf('cargo limits test: \t failed\n');
            else
                fprintf('cargo limits test: \t successeful\n');
            end
            e = obj.moveToCentroid();
            if(e == false)
                fprintf('move to centroid test: \t failed\n');
            else
                fprintf('move to centroid test: \t successeful\n');
            end

            [e1, e2] = obj.PointDensity();
            if(e1 == false)
                fprintf('point density first test: \t failed\n');
            else
                fprintf('point density first test: \t successeful\n');
            end
            if(e2 == false)
                fprintf('point density second test: \t failed\n');
            else
                fprintf('point density second test: \t successeful\n');
            end
        end
    end
end

