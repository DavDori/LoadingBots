classdef rect_load
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
            center_mass_absolute = obj.center + rotationMatrix(obj) * obj.center_mass;
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
            r(1,1:2) = obj.center + rotationMatrix(obj) * v1;
            r(2,1:2) = obj.center + rotationMatrix(obj) * v2; 
            r(3,1:2) = obj.center + rotationMatrix(obj) * v3; 
            r(4,1:2) = obj.center + rotationMatrix(obj) * v4; 
        end
        
        function R = rotationMatrix(obj)
            theta = obj.orientation;
            R = [cos(theta), -sin(theta);...
                 sin(theta), cos(theta)];
        end
        
        function pos = getLimitPosition(obj, pos_global_1, pos_global_2)
            pos_local_1 = rotationMatrix(obj)' * (pos_global_1 - obj.center);
            pos_local_2 = rotationMatrix(obj)' * (pos_global_2 - obj.center);
            pos = 0;
        end
        
        function flag = isInside(obj, absolute_point, offset) 
            % check weather a point is inside the rectangle or not
            local_point = rotationMatrix(obj)' * (absolute_point - obj.center);
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
            center_mass_absolute = (obj.center + rotationMatrix(obj) * obj.center_mass)';
            flag = false;
            if(n > 2) 
                for i = 1:n
                    for j = 1:n
                        for k = 1:n
                            if(k ~= i && k ~= j && j ~= i)
                                P1 = contact_points(i,:);
                                P2 = contact_points(j,:);
                                P3 = contact_points(k,:);
                                flag = inTriangle(center_mass_absolute,P1,P2,P3);
                                if(flag == true)
                                    break;
                                end
                            end
                        end
                    end
                end
            end
        end
        
    end
end

