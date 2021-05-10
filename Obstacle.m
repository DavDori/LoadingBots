classdef Obstacle < handle
    %OBSTACLE Summary of this class goes here
    %   Set as a circle to keep it simple
    
    properties
        radius % obsatcle radius
        center % position of the center
        speed  % current speed of the obj
        Ts     % simulation time
    end
    
    methods
        function obj = Obstacle(r, c, initial_v, Ts)
            %OBSTACLE Construct an instance of this class
            %   r -> obsatcle radius
            %   c -> obstacle center position
            %   initial_v -> initial velocity
            obj.radius = r;
            obj.center = c;
            obj.speed = initial_v;
            obj.Ts = Ts;
        end
        
        function move(obj)
            % using euler integration step, cheange the position of the
            % center of the obstacle
            obj.center = obj.center + obj.speed * obj.Ts;
        end
        
        
        function flag = isInside(obj, p)
            % check if a point is inside the obstacle
            flag = false; % outside
            if(distance2D(obj.center, p) <= obj.radius)
                flag = true;
            end
        end
        
        
        function plot(obj)
            hold on
            circle(obj.center(1), obj.center(2), obj.radius);
            hold off
        end
        
        
        function set.center(obj, p)
            obj.center = p;
        end
    end
end

