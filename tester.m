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
            robot_flock = flock(agents, cargo, obj.Ts, 0);
            
            robot_flock.computeVisibilitySets();
            robot_flock.computeVoronoiTessellation();
            robot_flock.applyConstantDensity();
            
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
            
            param_test.range = 1;          % [m] max observable range
            param_test.comm_range = 1.2;     % [m] max connection distance
            param_test.radius = 0.1;         % [m] hitbox of the agent
            param_test.N_rho = 30;           % division of the radius for discretization
            param_test.N_phi = 90;           % division of the angle for discretization
            
            agents(1) = agent('001', cargo.center, param_test, cargo, obj.map);
            
            robot_flock = flock(agents, cargo, obj.Ts, 0);
            sf = 1;
            % first test
            point = center + param_test.range / 2;
            
            robot_flock.computeVisibilitySets();
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
            % point outside the visibility range
            point_2 = center + param_test.range * 1.5;
            
            robot_flock.computeVisibilitySets();
            robot_flock.computeVoronoiTessellation();
            robot_flock.applySinglePointDensity(point_2, sf);
            cell = robot_flock.agents(1).Voronoi_cell.cell_density;
            [~, index] = max(cell(:));
            [i, j] = ind2sub(size(cell), index);
            
            rho = i * robot_flock.agents(1).Voronoi_cell.rho_res;
            phi = j * robot_flock.agents(1).Voronoi_cell.phi_res;
            
            phi_point = atan2(point_2(2) - center(2), point_2(1) - center(1));
            check_rho = rho >= param_test.range - obj.max_error;
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
            
            robot_flock = flock(agents, cargo, obj.Ts, 0);
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
            
            
            param_test.range = 1;          % [m] max observable range
            param_test.comm_range = 1.2;     % [m] max connection distance
            param_test.radius = 0.1;         % [m] hitbox of the agent
            param_test.N_rho = 30;           % division of the radius for discretization
            param_test.N_phi = 30;           % division of the angle for discretization
            dist = param_test.radius * 1.1;
            agents(1) = agent('001', [2 + dist; 0], param_test, cargo, obj.map);
            agents(2) = agent('002', [2 - dist; 0], param_test, cargo, obj.map);
            
            robot_flock = flock(agents, cargo, obj.Ts, 0);
            
            robot_flock.meetNeighbours();
            robot_flock.computeVisibilitySets();
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
                robot_flock.plotVoronoiTessellationDetailed(1);
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
            
            default_param = obj.param;
            default_param.N_phi = 140;
            default_param.N_rho = 140;
            default_param.range = 1;
            % cargo has to be a rectangle
            cargo = rect_load(center, center_mass, orientation, dimensions);
            vertex = computeVertexPositions(cargo); 
            % set the agent at 1 vertex
            agents(1) = agent('001', vertex(1,:), default_param, cargo, obj.map);
            robot_flock = flock(agents, cargo, obj.Ts, 0);
            
            robot_flock.meetNeighbours();
            robot_flock.computeVisibilitySets();
            
            robot_flock.computeVoronoiTessellationCargo(offset);
            % get the points in the cell that are set to 1
            p = sum(robot_flock.agents.Voronoi_cell.visibility_set, 'all');
            % get the total amount of points
            p_tot = robot_flock.agents.Voronoi_cell.rho_n * robot_flock.agents.Voronoi_cell.phi_n;
            
            flag = (abs(p / p_tot - 1/4) < obj.max_error);
            % true -> test passed
            % false -> test failed
        end
        
        
        function flag = doubleDensityOnCell(obj, view)
            center = [2 ; 2]; % [m]
            center_mass = [0;0];        % [m]
            dimensions = [1.5; 1];      % [m]
            orientation = pi/2;         % [rad]
            cargo = rect_load(center, center_mass, orientation, dimensions);
            param_test.range = 1;          % [m] max observable range
            param_test.comm_range = 1.2;     % [m] max connection distance
            param_test.radius = 0.1;         % [m] hitbox of the agent
            param_test.N_rho = 30;           % division of the radius for discretization
            param_test.N_phi = 45;           % division of the angle for discretization
            
            agents(1) = agent('001', cargo.center, param_test, cargo, obj.map);
            
            robot_flock = flock(agents, cargo, obj.Ts, 0);
            sf = 0.5;
            % first test
            point1 = center + param_test.range / 2;
            point2 = center - param_test.range / 2;
            
            robot_flock.computeVisibilitySets();
            robot_flock.computeVoronoiTessellation();
            robot_flock.applySinglePointDensity(point1, sf);
            robot_flock.applySinglePointDensity(point2, sf);
            robot_flock.computeVoronoiCentroids();
            % centroid expected on the robot location
            error_x = robot_flock.agents.Voronoi_cell.centroid(1);
            error_y = robot_flock.agents.Voronoi_cell.centroid(2);
            flag = abs(error_x) < obj.max_error && abs(error_y) < obj.max_error;
            
            if(view == true)
                figure()
                grid on
                hold on
                axis equal
                show(obj.map)
                robot_flock.plot();
                robot_flock.plotVoronoiTessellationDetailed(1);
            end
        end
        
        
        function flag = scanComm(obj)
            % check if the agents recive the scanned area 
            center = [2 ; 2]; % [m]
            center_mass = [0;0];        % [m]
            dimensions = [1.5; 1];      % [m]
            orientation = pi/2;         % [rad]
            
            param_test.range = 1;          % [m] max observable range
            param_test.comm_range = 1.2;     % [m] max connection distance
            param_test.radius = 0.1;         % [m] hitbox of the agent
            param_test.N_rho = 60;           % division of the radius for discretization
            param_test.N_phi = 90;           % division of the angle for discretization
            
            cargo = rect_load(center, center_mass, orientation, dimensions);
            pos = cargo.center;
            agents(1) = agent('001', pos , param_test, cargo, obj.map);
            pos = pos - [0; param_test.range / 2]; % in range of 1
            agents(2) = agent('002', pos, param_test, cargo, obj.map);
            pos = pos + [0; param_test.comm_range * 2]; % out of range of both 1,2
            agents(3) = agent('003', pos, param_test, cargo, obj.map);
            
            robot_flock = flock(agents, cargo, obj.Ts, 0);
            
            robot_flock.meetNeighbours(); 
            robot_flock.sendScan();
            flag1 = isempty(robot_flock.agents(3).Neighbours_scan) == true;
            flag2 = isempty(robot_flock.agents(2).Neighbours_scan) == false;
            flag3 = isempty(robot_flock.agents(1).Neighbours_scan) == false;
            
            flag = flag1 && flag2 && flag3;
        end
        
        
        function flag = connectivityMaintenance(obj, view)
            map_cm = png2BOMap('map_test_1.png', 22); % specific map to check connectivity maintenance
            center = [map_cm.XWorldLimits(2) / 2 ; map_cm.YWorldLimits(2) / 2]; % [m]
            center_mass = [0;0];        % [m]
            dimensions = [1.5; 1];      % [m]
            orientation = pi/2;         % [rad]
            test_param = obj.param;
            test_param.range = 1;
            test_param.N_phi = 40;
            test_param.N_rho = 40;
            
            cargo = rect_load(center, center_mass, orientation, dimensions);
            pos = center + [0; 0.6];
            agents(1) = agent('001', pos, test_param, cargo, map_cm);
            pos = center - [-0.5; 0.2]; % in range of 1
            agents(2) = agent('002', pos, test_param, cargo, map_cm);
            pos = center - [0.5; 0.2]; % in range of 1
            agents(3) = agent('003', pos, test_param, cargo, map_cm);
            
            robot_flock = flock(agents, cargo, obj.Ts, 0);
            robot_flock.meetNeighbours(); % meat neighbours
            robot_flock.sendScan(); % send scan
            robot_flock.computeVisibilitySets();
            robot_flock.connectivityMaintenance();
            robot_flock.computeVoronoiTessellation();
            robot_flock.applyConstantDensity();
            robot_flock.computeVoronoiCentroids();
            
            if(view == true)
                figure()
                hold on
                robot_flock.plotVoronoiTessellationDetailed(1)
                robot_flock.plot();
                robot_flock.plotCentroids();
            end
            
            flag_x = abs(agents(1).Voronoi_cell.centroid(1)) < obj.max_error;
            flag_y = agents(1).Voronoi_cell.centroid(2) < agents(1).position(2);
            
            flag = flag_x && flag_y;
        end
        
        
        function flag = reachWayPoint(obj, view)
            center = [obj.map.XWorldLimits(2) / 2 ; obj.map.YWorldLimits(2) / 2]; % [m]
            center_mass = [0;0];        % [m]
            dimensions = [1.5; 1];      % [m]
            orientation = pi/2;         % [rad]
            param_test.range = 2;          % [m] max observable range
            param_test.comm_range = 5;     % [m] max connection distance
            param_test.radius = 0.1;         % [m] hitbox of the agent
            param_test.N_rho = 60;           % division of the radius for discretization
            param_test.N_phi = 90;           % division of the angle for discretization
            cargo = rect_load(center, center_mass, orientation, dimensions);
            
            agents(1) = agent('001', [obj.map.XWorldLimits(2) / 2 + 0.5; 2.5], param_test, cargo, obj.map);
            agents(2) = agent('002', [obj.map.XWorldLimits(2) / 2 + 0.5; 1.5], param_test, cargo, obj.map);
            agents(3) = agent('003', [obj.map.XWorldLimits(2) / 2 - 0.5; 1.5], param_test, cargo, obj.map);
            agents(4) = agent('004', [obj.map.XWorldLimits(2) / 2 - 0.5; 2.5], param_test, cargo, obj.map);
            
            robot_flock = flock(agents, cargo, obj.Ts, 0);
            
            u = [0.3,0]; % move 30 centimeters upwards 
            dest = robot_flock.setTrajectory(u);
            robot_flock.setWayPoints(dest);
            
            if(view == true)
                figure()
                hold on
                robot_flock.plot();
                robot_flock.plotAgentsPath(dest);
            end
            exit = false;
            e = 0.05; % distance from waypoints that every agent has to reach
            n = 1;
            % if the agents don't reach the waypoint in less than 30 steps,
            % the test fails
            while(exit == false && n < 30)
                % communication
                robot_flock.meetNeighbours(); % meat neighbours
                robot_flock.sendScan(); % send scan
                % visibility set and Voronoi
                robot_flock.computeVisibilitySets();
                robot_flock.connectivityMaintenance();
                robot_flock.computeVoronoiTessellation();
                robot_flock.applyWayPointDensity(0.2);
                robot_flock.computeVoronoiCentroids();
                % movement
                robot_flock.moveToCentroids(1);
                reached = robot_flock.areWayPointsReached(e);
                exit = all(reached);
                n = n + 1;
            end
            
            if(view == true)
                figure()
                hold on
                grid on
                robot_flock.plotVoronoiTessellationDetailed(1)
                robot_flock.plot();
                robot_flock.plotCentroids();
            end
            
            flag = reached;
        end
        
        
        function flag = fixedFormation(obj, flag_plot)
            % Test the collision detection with two agents, their centroids
            % should move in opposite directions and their Voronoi cells
            % should show a gap instead of touching
            
            center = [2 ; 2]; % [m]
            center_mass = [0;0];        % [m]
            dimensions = [1.5; 1];      % [m]
            orientation = pi/2;         % [rad]
            cargo = rect_load(center, center_mass, orientation, dimensions);
            
            
            param_test.range = 3;          % [m] max observable range
            param_test.comm_range = 5;     % [m] max connection distance
            param_test.radius = 0.1;         % [m] hitbox of the agent
            param_test.N_rho = 60;           % division of the radius for discretization
            param_test.N_phi = 60;           % division of the angle for discretization
            
            agents(1) = agent('001', [2 + 1; 0], param_test, cargo, obj.map);
            agents(2) = agent('002', [2 - 1; 0], param_test, cargo, obj.map);
            
            robot_flock = flock(agents, cargo, obj.Ts, 0);
            robot_flock.fixFormation();
            robot_flock.meetNeighbours();
            robot_flock.sendScan();
            robot_flock.computeVisibilitySets();
            robot_flock.connectivityMaintenance();
            robot_flock.computeVoronoiTessellationFF(0.1);
            robot_flock.applyConstantDensity();
            robot_flock.computeVoronoiCentroids();

            if(flag_plot == true)
                figure()
                grid on
                hold on
                axis equal
                robot_flock.plotVoronoiTessellationDetailed(1);
                robot_flock.plot()
                robot_flock.plotCentroids();
                hold off
            end
            
            flag = true;
        end
        
        
        function runAll(obj, view)
            e = obj.centroidOmogeneus();
            if(e == false)
                fprintf('centroid test: \t failed\n');
            else
                fprintf('centroid test: \t successeful\n');
            end
            e = obj.collisionDetection(view);
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
            e = obj.doubleDensityOnCell(view);
            if(e == false)
                fprintf('double point density test: \t failed\n');
            else
                fprintf('double point density test: \t successeful\n');
            end
            e = obj.scanComm();
            if(e == false)
                fprintf('communication of scan area test: \t failed\n');
            else
                fprintf('communication of scan area test: \t successeful\n');
            end
            e = obj.connectivityMaintenance(view);
            if(e == false)
                fprintf('connectivity maintenance test: \t failed\n');
            else
                fprintf('connectivity maintenance test: \t successeful\n');
            end
            e = obj.reachWayPoint(view);
            if(e == false)
                fprintf('reach way point test: \t failed\n');
            else
                fprintf('reach way point test: \t successeful\n');
            end
        end
    end
end

