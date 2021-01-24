classdef agent < handle
    %AGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name
        position (2,1) double {mustBeNumeric}   % absolute coordinates
        dimension (1,1) double {mustBeNumeric}  % agent as a circle 
        attached (1,1) logical % is the agent attached to the load? 
        load_box rect_load
        Ts (1,1) double {mustBeNumeric} %sampling time
        comm_range (1,1) double {mustBeNumeric}
        lidar_range (1,1) double {mustBeNumeric}
        msg_in
    end
    
    methods
        function obj = agent(name, init_position, param, box)
            obj.name = name;
            obj.position = init_position;
            obj.load_box = box;
            obj.dimension = param.radius;
            obj.comm_range = param.comm_range;
            obj.lidar_range = param.range;
            obj.attached = false; % starts as unattached
            obj.msg_in = [];
        end
        
        % SETTERS
        function set.attached(obj, v)
            obj.attached = v;
        end
        function set.msg_in(obj, text)
            obj.msg_in = text;
        end
        function set.position(obj, pos)
            obj.position = pos;
        end
        function set.Ts(obj, t)
            obj.Ts = t;
        end
        
        function clearComms(obj)  % clear the comm buffer
            obj.msg_in = [];
        end
        
        function move(obj, velocity) % compute the position at the next integration step               
            if(size(velocity, 2) ~= 1)
                obj.position = obj.position + obj.Ts * velocity';
            else
                obj.position = obj.position + obj.Ts * velocity;
            end
        end
        
        function obj = attach(obj) % if possible attach the robot to the load
            obj.attached = isInsideRectLoad(obj);
            if(obj.attached == false)
                error('Attach operation outside the load area')
            end
        end
        
        function obj = detach(obj) % detach the agent from the load
            obj.attached = false;
        end
        
        
        function r = isInsideRectLoad(obj) % check weather the agent is within the designated area
            r = isInside(obj.load_box, obj.position);
        end
        
        function sendMessage(obj, other, text) % send a message to the other agent if it's within comm distance
            q = obj.position - other.position;
            dist = sqrt(q' * q);
            if(obj.comm_range > dist)
                other.msg_in = strcat(other.msg_in, text);
            end
        end
        
        function plot(obj) % represent the agent on a 2D plain in red if free to move, in blue if attached
            hold on
            if(obj.attached == true)
                plot(obj.position(1), obj.position(2), 'ob')
            else
                plot(obj.position(1), obj.position(2), 'or')
            end
            hold off
        end
    end
end

