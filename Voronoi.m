classdef Voronoi < handle
    %VORONOI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        visibility_set {mustBeNumeric} % points seen 
        cell_density {mustBeNumeric} % density of each element of the cell
        
        centroid {mustBeNumeric}
        rho_res {mustBeNumeric}
        phi_res {mustBeNumeric}
        rho_n {mustBeNumeric}
        phi_n {mustBeNumeric}
    end
    
    methods
        
        function obj = Voronoi(rho_n, phi_n, max_range)
            %VORONOI Construct an instance of this class
            
            obj.rho_n = rho_n;
            obj.phi_n = phi_n;
            [obj.rho_res, obj.phi_res] = getCellResolution(obj, max_range);
        end
        
        
        function [rho_res, phi_res] = getCellResolution(obj, max_range)
            % calculate the resolution for the Voronoi cell
            rho_res = max_range / obj.rho_n;
            phi_res = 2 * pi / obj.phi_n;
        end
        
        
        function visibilitySet(obj, agent_scan)
            % discretize a circular space around the agent and define the
            % visibility set of the agent
            initVisibilitySet(obj);
            initCells(obj);
            tmp_cell = obj.visibility_set;
            % for every point in the cell check if it's closer to agent
            for i = 1:obj.phi_n % for every angle
                distance_index = ceil(agent_scan(i,1) / obj.rho_res);
                for j = 1:distance_index % for some radius                   
                    tmp_cell(j,i) = 1;
                end
            end
            obj.visibility_set = tmp_cell;
        end
        
        function unionVisibilitySets(obj, pos, range_max, Neighbours, Neighbours_scans)
            % takes the visibility set that has to be already computed and
            % intersects it with the visibility set of the neighbours
            % NOTE: lidar range assumed to be the same for each agent
            tmp_cell = obj.visibility_set;
            % for every point 
            for i = 1:obj.rho_n % for every angle
                for j = 1:obj.phi_n % for some radius
                    if(obj.visibility_set(i,j) == 1)
                        % compute the point in the absolute ref. frame
                        rho = i * obj.rho_res;
                        phi = j * obj.phi_res;
                        [x, y] = polar2cartesian(rho, phi);
                        point = [x; y] + pos;
                        for k = 1:length(Neighbours_scans) % for every scan
                            % transfers the coordinates to the
                            % reference frame of the Neighbour
                            q = point - Neighbours(k).position';
                            dist = sqrt(q'*q); % distance between point and neighbour
                            if(dist > range_max(k))
                                tmp_cell(i,j) = 0;
                                break;
                            else
                                % assumption: every neighbour discretize
                                % the area in the same way.
                                % angle has to in 0 and 2pi
                                angle = change_piTo2pi(atan2(q(2),q(1)));
                                % find the curesponding index
                                index = obj.getAngleIndex(angle);
                                measured_rho = Neighbours_scans(k).scan(index, 1);
                                if(measured_rho < dist) % point is outside the visibility set of the neighbour
                                    tmp_cell(i,j) = 0;
                                    break
                                end
                            end
                        end
                        
                    end
                end
            end
            obj.visibility_set = tmp_cell;
        end
        
        
        function voteVisibilitySets(obj, pos, range_max, Neighbours, Neighbours_scans)
            % takes the visibility set that has to be already computed and
            % intersects it with the visibility set of the neighbours, the
            % more a point is shared between agents, the higher its value
            % NOTE: lidar range assumed to be the same for each agent
            val = 1 / length(Neighbours);
            tmp_cell = obj.visibility_set;
            % for every point 
            for i = 1:obj.rho_n % for every angle
                for j = 1:obj.phi_n % for some radius
                    if(obj.visibility_set(i,j) == 1)
                        % compute the point in the absolute ref. frame
                        rho = i * obj.rho_res;
                        phi = j * obj.phi_res;
                        [x, y] = polar2cartesian(rho, phi);
                        point = [x; y] + pos;
                        for k = 1:length(Neighbours_scans) % for every scan
                            % transfers the coordinates to the
                            % reference frame of the Neighbour
                            q = point - Neighbours(k).position';
                            dist = sqrt(q'*q); % distance between point and neighbour
                            if(dist > range_max(k))
                                tmp_cell(i,j) = tmp_cell(i,j) - val;
                                break;
                            else
                                % assumption: every neighbour discretize
                                % the area in the same way.
                                % angle has to in 0 and 2pi
                                angle = change_piTo2pi(atan2(q(2),q(1)));
                                % find the curesponding index
                                index = obj.getAngleIndex(angle);
                                measured_rho = Neighbours_scans(k).scan(index, 1);
                                if(measured_rho < dist) % point is outside the visibility set of the neighbour
                                    % the value is reduced by val for every
                                    % neighbour that don't share that point
                                    % of the cell
                                    tmp_cell(i,j) = tmp_cell(i,j) - val;
                                    if(tmp_cell(i,j) < 0)
                                        tmp_cell(i,j) = 0;
                                    end
                                    break
                                end
                            end
                        end
                        
                    end
                end
            end
            obj.visibility_set = tmp_cell;
        end
        
        
        function computeCell(obj, pos, Neighbours, min_agents_dist)
            if(isempty(obj.visibility_set))
                error('Visibility set has to be computed before this operation');
            end
            
            % initCells(obj);
            tmp_cell = obj.visibility_set;
            % for every point 
            for i = 1:obj.rho_n % for every angle
                for j = 1:obj.phi_n % for some radius
                    if(obj.visibility_set(i,j) > 0)
                        % compute the point in the absolute ref. frame
                        inside = obj.pointDomain(i, j, pos, Neighbours, min_agents_dist);
                        if(inside == false)
                            tmp_cell(i,j) = 0;
                        end
                    end
                end
            end
            obj.visibility_set = tmp_cell;
        end
        
        
        function inside = pointDomain(obj, i, j, pos, Neighbours, min_agents_dist)
            inside = true;
            point = obj.getPoint(i,j) + pos;
            % distance between point and agent
            dist_point_agent = distance2D(point, pos);
            multiple_values = length(min_agents_dist) > 1;
            
            for k = 1:length(Neighbours) % for every neighbour
                if(multiple_values == true)
                    th = min_agents_dist(k);
                else
                    th = min_agents_dist;
                end
                % distance between agent and neighbour
                dist_agent_neighbour = distance2D(pos, Neighbours(k).position');

                if(th > dist_agent_neighbour / 2)
                    factor = 2 * th / dist_agent_neighbour - 1;
                    delta_pos = (pos - Neighbours(k).position') * factor;
                    neighbour_pos = Neighbours(k).position' + delta_pos;
                else
                    neighbour_pos = Neighbours(k).position';
                end
                % modifies distance 
                dist_point_neighbour = distance2D(point, neighbour_pos);
                if(dist_point_neighbour < dist_point_agent)
                    inside = false;
                    break;
                end
            end
        end
        
        
        function point = getPoint(obj, i, j)
            % converts a position in the cell into a point in the local
            % reference frame
            rho = i * obj.rho_res;
            phi = j * obj.phi_res;
            [x, y] = polar2cartesian(rho, phi);
            point = [x; y];
        end
        
            
        function index = getAngleIndex(obj, angle)
            % compute the approximation of the angle and the corresponding
            % index in order to compare it with the scan / cell 
            index = round(angle / obj.phi_res);
            if(index == 0)
                index = obj.phi_n;
            end
        end  
        
        
        function tmp_cell = applyCargoLimits(obj, agent_position, cargo, offset)
            % for every point in the cell check if it's in the cargo domain
            % the cell value is kept, otherwise it's set to 0
            tmp_cell = obj.visibility_set;
            
            for i = 1:obj.phi_n % for every angle
                for j = 1:obj.rho_n % for every radius in the visibility set
                    rho = j * obj.rho_res;
                    phi = i * obj.phi_res;
                    [x, y] = polar2cartesian(rho, phi);
                    point = [x; y];
                    if(tmp_cell(j,i) ~= 0 && isInside(cargo, agent_position + point, offset) == false)
                        % outside the perimeter
                        tmp_cell(j,i) = 0;
                    end
                end
            end
            obj.visibility_set = tmp_cell;
        end
        
        
        function tmp_cell = applyDensity(obj, fun_d)
            % apply the desnsity function on every point of the Voronoi
            % cell and return the resulting cell 
            tmp_cell = obj.visibility_set;
            
            % for every point apply the density function
            for i = 1:obj.phi_n % for every angle
                for j = 1:obj.rho_n % for some radius
                    if(obj.visibility_set(j,i) > 0)
                        rho = j * obj.rho_res;
                        phi = i * obj.phi_res;
                        raw_density = fun_d(rho, phi);
                        tmp_cell(j,i) = raw_density * obj.visibility_set(j,i);
                    end
                end
            end
            % calculate the normalizer for the applied density
            %norm = sum(tmp_cell(:)); 
            if(isempty(obj.cell_density) == true) 
                % initailizes 
                obj.cell_density = tmp_cell;
            else
                % add
                obj.cell_density = obj.cell_density + tmp_cell;
            end
        end
        
        
        function addConstantDensity(obj, c)
            % the calculation of the centroid depends on the cell_density.
            % In the case of no specified density o constant density this
            % function is used
            if(nargin == 1)
                c = 1;  %default value
            end
            tmp_cell = obj.visibility_set * c;
            
            if(isempty(obj.cell_density) == true)
                obj.cell_density = tmp_cell;
            else
                obj.cell_density = obj.cell_density + tmp_cell;
            end
        end
        
        
        function mass = computeCellMass(obj, fun_d)
            % approximation of every point as a trapezoid
            mass = 0;
            
            for i = 1:obj.rho_n % for every radius
                for j = 1:obj.phi_n % for every angle
                    rho = obj.rho_res * i;
                    phi = obj.phi_res * j;
                    s_base = 2 * (rho - obj.rho_res / 2) * sin(obj.phi_res / 2);
                    b_base = 2 * (rho + obj.rho_res / 2) * sin(obj.phi_res / 2);
                    density = fun_d(rho, phi) * obj.cell_density(i,j);
                    mass = mass + density * (s_base + b_base) * obj.rho_res / 2;
                end
            end   
        end
        
        
        function c = computeCentroid(obj)
            % compute the centroid of the Voronoi cell considering a
            % mass function
           
            fun_x = @(rho,phi) rho * cos(phi);
            fun_y = @(rho,phi) rho * sin(phi);
            fun_m = @(rho,phi) 1;
            
            mass = computeCellMass(obj, fun_m);
            if(mass == 0)
                c = [0;0];
            else
                x = computeCellMass(obj, fun_x) / mass;
                y = computeCellMass(obj, fun_y) / mass;
                c = [x;y]; % c corresponds with the center of mass of the cell
            end
            obj.centroid = c;
        end
        
        
        function initCells(obj)
            % initialize the voronoi cell with zeros
            obj.cell_density = zeros(obj.rho_n, obj.phi_n);
        end
        
        
        function initVisibilitySet(obj)
            obj.visibility_set = zeros(obj.rho_n, obj.phi_n);
        end
        
        
        function setDensityAngle(obj, angle_range, value_in, value_out)
            % given an angle range it multiply all density cells inside by 
            % value_in and the others by value_out
            index_lower = obj.getAngleIndex(change_piTo2pi(angle_range(1)));
            index_upper = obj.getAngleIndex(change_piTo2pi(angle_range(2)));
            
            density_mask = zeros(size(obj.cell_density));
            [rows, cols] = size(obj.cell_density);
            
            if(index_lower > index_upper) % select the opposite area
                density_mask(:, 1:index_upper) = ...
                    ones(rows, index_upper) * value_in;
                density_mask(:, index_upper+1:index_lower) = ...
                    ones(rows, index_lower - index_upper) * value_out;
                density_mask(:, index_lower+1:end) = ...
                    ones(rows, cols - index_lower) * value_in;
            else
                density_mask(:, 1:index_lower) = ...
                    ones(rows, index_lower) * value_out;
                density_mask(:, index_lower+1:index_upper) = ...
                    ones(rows, index_upper - index_lower) * value_in;
                density_mask(:, index_upper+1:end) = ...
                    ones(rows, cols - index_upper) * value_out;
            end
            
            if(sum(sum(obj.cell_density)) == 0)
                obj.cell_density = obj.visibility_set .* density_mask;
            else
                obj.cell_density = obj.cell_density .* density_mask;
            end
        end
        
        
        function plot(obj, position, step)
            % plot a detaild version of the voronoi cell considering a
            % density function applied on it
            hold on
            max_value =  max(obj.cell_density, [], 'all');
            if(max_value > 0)
                for i = 1:step:obj.rho_n % rho
                    for j = 1:step:obj.phi_n % phi
                        % represent every point of cell
                        if(obj.visibility_set(i,j) > 0)
                            [x_local,y_local] = polar2cartesian(i * obj.rho_res, j * obj.phi_res);
                            local_p = [x_local, y_local]';
                            global_p = local2global(position, local_p);

                            normalizer = obj.cell_density(i,j) / max_value;
                            color = [1,1,1] - [1,1,1] * normalizer;
                            plot(global_p(1),global_p(2), '.', 'Color', color);
                        end
                    end
                end
                hold off
            end
        end
        
        
        function plotFast(obj, position, color)
            % consider only the outter perimeter of the cell
            edges = zeros(obj.phi_n, 2); 
            
            for i = 1:obj.phi_n % for every angle
                [x_local,y_local] = polar2cartesian(...
                    sum(obj.visibility_set(:,i) ~= 0) * obj.rho_res, i * obj.phi_res);
                local_p = [x_local, y_local]';
                edges(i,:) = local2global(position, local_p);
            end
            plot(edges(:,1), edges(:,2), 'Color', color);
        end
        
        % SETTERS methods
        function set.visibility_set(obj, v_cell)
            obj.visibility_set = v_cell;
        end
        function set.cell_density(obj, v_cell)
            obj.cell_density = v_cell;
        end
        function set.centroid(obj, c)
            obj.centroid = c;
        end
    end
end

