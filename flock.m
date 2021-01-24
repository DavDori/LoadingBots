classdef flock < handle
    %FLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        agents agent
        my_load rect_load
        n_agents (1,1) double {mustBeNumeric}
    end
    
    methods
        function obj = flock(agents_array, box, sampling_time)
            %FLOCK 
            obj.agents = agents_array;
            obj.n_agents = length(agents_array);
            obj.my_load = box;
            for a = obj.agents
                a.Ts = sampling_time;
            end
        end
        
        function moveAgent(obj,index, velocity)
            if(index < obj.n_agents)
                obj.agents(index).move(velocity);
            end
        end
        
        function flag = checkBalance(obj)
            % check if the load should be falling
            points = zeros([obj.n_agents, 2]);
            n = 1;
            for i = 1:nodes
                if(obj.agents(i).attached == true) % considers only attached agents
                    points(n,:) = obj.agents(i).position';
                    n = n + 1;
                end
            end
            flag = obj.my_load.isBalanced(points);
        end
        
        function plot(obj)
            % representation in a 2D plane of the agents and the load
            hold on
            axis equal
            obj.my_load.plot('r')
            for a = obj.agents
                a.plot();
            end
            hold off
        end
    end
end

