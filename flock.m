classdef flock
    %FLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        agents agent
        connections connection
        n_agents (1,1) double {mustBeNumeric}
    end
    
    methods
        function obj = flock(agents_array)
            %FLOCK 
            obj.agents = agents_array;
            obj.connections = setUpNetwork(agents_array);
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

