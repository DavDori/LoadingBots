classdef rect_load < handle
    %LOAD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        center (2,1) double {mustBeNumeric} % in absolute coordinates
        center_mass (2,1) double {mustBeNumeric} % in relative coordinates
        orientation (1,1) double {mustBeNumeric}
        dimension (2,1) double {mustBeNumeric}
    end
    
    methods
        function obj = rect_load(c,c_mass, initial_angle, dim)
            obj.center = c;
            obj.center_mass = c_mass;
            obj.orientation = initial_angle;
            obj.dimension = dim;
        end
        
        function plot(obj, cmd) % draw the rectangle and center of mass
            hold on
            v = obj.computeVertexPositions();
            center_mass_absolute = obj.center + rotationMatrix(obj.orientation) * obj.center_mass;
            plot([v(1,1),v(2,1)], [v(1,2),v(2,2)], cmd);
            plot([v(2,1),v(3,1)], [v(2,2),v(3,2)], cmd);
            plot([v(3,1),v(4,1)], [v(3,2),v(4,2)], cmd);
            plot([v(4,1),v(1,1)], [v(4,2),v(1,2)], cmd);
            plot(center_mass_absolute(1), center_mass_absolute(2), 'og')
            
            hold off
        end
        
        function r = computeVertexPositions(obj) % calculate the vertex position of the rectangle
            v1 = [-obj.dimension(1) / 2, -obj.dimension(2) / 2]';
            v2 = [-obj.dimension(1) / 2, +obj.dimension(2) / 2]';
            v3 = [+obj.dimension(1) / 2, +obj.dimension(2) / 2]';
            v4 = [+obj.dimension(1) / 2, -obj.dimension(2) / 2]';
            r(1,1:2) = obj.center + rotationMatrix(obj.orientation) * v1;
            r(2,1:2) = obj.center + rotationMatrix(obj.orientation) * v2; 
            r(3,1:2) = obj.center + rotationMatrix(obj.orientation) * v3; 
            r(4,1:2) = obj.center + rotationMatrix(obj.orientation) * v4; 
        end
                
%         function pos = getLimitPosition(obj, pos_global_1, pos_global_2)
%             pos_local_1 = rotationMatrix(obj.orientation)' * (pos_global_1 - obj.center);
%             pos_local_2 = rotationMatrix(obj.orientation)' * (pos_global_2 - obj.center);
%         end
        
        function flag = isInside(obj, absolute_point, offset) 
            % check weather a point is inside the rectangle or not
            local_point = rotationMatrix(obj.orientation)' * (absolute_point - obj.center);
            inVertical = local_point(2) < offset + obj.dimension(2) / 2 &&...
                         local_point(2) > -offset - obj.dimension(2) / 2; 
            inOrizontal = local_point(1) < offset + obj.dimension(1) / 2 &&...
                          local_point(1) > -offset - obj.dimension(1) / 2;
            flag = inVertical && inOrizontal;
        end
        
        function flag = isBalanced(obj, contact_points)
            % check if at least a triangle made with some contact points
            % has inside the center of mass.
            n = size(contact_points,1); % number of contact points
            center_mass_absolute = (obj.center + rotationMatrix(obj.orientation) * obj.center_mass)';
            flag = false;
            if(n > 2)
                set_i = 1:n;
                for i = set_i
                    set_j = set_i(set_i ~= i); % all exept i
                    for j = set_j 
                        set_k = set_j(set_j ~= j); % all exept i and j
                        for k = set_k
                            P1 = contact_points(i,:);
                            P2 = contact_points(j,:);
                            P3 = contact_points(k,:);
                            flag = inTriangle(center_mass_absolute,P1,P2,P3);
                            if(flag == true)
                                % if for at least one triangle the
                                % baricenter is inside, the cargo is in
                                % balance
                                return;
                            end
                        end
                    end
                end
            end
        end
        
        function move(obj, u, ts)
            % the input u is composed by a vector [linear velocity, angular
            % velocity]. The object model is that of a unicycle
            dot = unicycleModel(u , [obj.center; obj.orientation]);
            obj.center = obj.center + dot(1:2) * ts;
            obj.orientation = obj.orientation + dot(3) * ts;
        end
        
    end
end

