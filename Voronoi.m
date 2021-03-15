classdef Voronoi < handle
    %VORONOI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        visibility_set {mustBeNumeric} % points seen 
        cell_density {mustBeNumeric} % density of each element of the cell
        cell_tessellaion {mustBeNumeric} % binary vision of the cell
        
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
        
        
        function computeCell(obj, Neighbours, agent_size)
            % discretize a circular space around the agent. Set to 1 the
            % point closer to the agent and to 0 those closer to the
            % neighbour agents.
            % NOTE: neighbours have to be defined in the local refernce
            % frame relative to the current agent.
            % define resolution of the angle and radius.
            % NOTE: first a visibility set has to be computed
            if(isempty(obj.visibility_set))
                error('Visibility set has to be computed before this operation');
            end
            initCells(obj);
            tmp_cell = obj.visibility_set;
            % for every point in the cell check if it's closer to agent
            for i = 1:obj.phi_n % for every angle
                for j = 1:sum(obj.visibility_set(:,i)) % for some radius in the visibility set
                    rho = j * obj.rho_res;
                    phi = i * obj.phi_res;
                    [x, y] = polar2cartesian(rho, phi);
                    point = [x; y];
                    
                    tmp_cell(j,i) = isCloser(Neighbours, point, agent_size);
                end
            end
            obj.cell_tessellaion = tmp_cell;
        end
        
        
        function visibilitySet(obj, agent_scan)
            % discretize a circular space around the agent and define the
            % visibility set of the agent
            initVisibilitySet(obj);
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
                            if(dist > range_max)
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
            tmp_cell = obj.cell_tessellaion;
            
            for i = 1:obj.phi_n % for every angle
                for j = 1:obj.rho_n % for every radius in the visibility set
                    rho = j * obj.rho_res;
                    phi = i * obj.phi_res;
                    [x, y] = polar2cartesian(rho, phi);
                    point = [x; y];
                    if(isInside(cargo, agent_position + point, offset) == false)
                        % outside the perimeter
                        tmp_cell(j,i) = 0;
                    end
                end
            end
            obj.cell_tessellaion = tmp_cell;
        end
        
        
        function tmp_cell = applyDensity(obj, fun_d)
            % apply the desnsity function on every point of the Voronoi
            % cell and return the resulting cell 
            tmp_cell = obj.cell_tessellaion;
            
            % for every point in the cell check if it's closer to agent
            for i = 1:obj.phi_n % for every angle
                for j = 1:obj.rho_n % for some radius
                    rho = j * obj.rho_res;
                    phi = i * obj.phi_res;
                    tmp_cell(j,i) = fun_d(rho, phi) * obj.cell_tessellaion(j,i);
                end
            end
            % calculate the normalizer for the applied density
            norm = sum(tmp_cell(:)); 
            % update the current density
            obj.cell_density = obj.cell_density + tmp_cell / norm;
        end
        
        
        function cell_tmp = addConstantDensity(obj, c)
            % the calculation of the centroid depends on the cell_density.
            % In the case of no specified density o constant density this
            % function is used
            if(nargin == 1)
                c = 1;  %default value
            end
            cell_tmp = obj.cell_tessellaion * c;
            obj.cell_density = obj.cell_density + cell_tmp;
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
            
            x = computeCellMass(obj, fun_x) / mass;
            y = computeCellMass(obj, fun_y) / mass;
            c = [x;y]; % c corresponds with the center of mass of the cell
            obj.centroid = c;
        end
        
        
        function initCells(obj)
            % initialize the voronoi cell with zeros
            obj.cell_tessellaion = zeros(obj.rho_n, obj.phi_n);
            obj.cell_density = zeros(obj.rho_n, obj.phi_n);
        end
        
        
        function initVisibilitySet(obj)
            obj.visibility_set = zeros(obj.rho_n, obj.phi_n);
        end
        
        
        function plot(obj, position, step)
            % plot a detaild version of the voronoi cell considering a
            % density function applied on it
            hold on
            max_value =  max(obj.cell_density, [], 'all');
            if(max_value == 0)
                max_value = 1;
            end
            for i = 1:step:obj.rho_n % rho
                for j = 1:step:obj.phi_n % phi
                    % represent every point of cell
                    if(obj.cell_tessellaion(i,j) == 1)
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
        
        
        function plotFast(obj, position, color)
            % consider only the outter perimeter of the cell
            edges = zeros(obj.phi_n, 2); 
            
            for i = 1:obj.phi_n % for every angle
                [x_local,y_local] = polar2cartesian(...
                    sum(obj.cell_tessellaion(:,i) ~= 0) * obj.rho_res, i * obj.phi_res);
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
        function set.cell_tessellaion(obj, tes)
            obj.cell_tessellaion = tes;
        end
    end
end

