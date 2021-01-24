clear all
clc

%% SET UP

% load
center = [1;1];             % [m]
center_mass = [0.1;0.1];    % [m]
dimensions = [1.5; 1];      % [m]
orientation = pi/4;         % [rad]

% Agents
param.range = 1;            % [m] max observable range
param.comm_range = 0.8;     % [m] max connection distance
param.radius = 0.1;         % [m] hitbox of the agent

loadObj = rect_load(center, center_mass, orientation, dimensions);
agents(1) = agent('James',[1.2;1.3], param, loadObj);
agents(2) = agent('Yasuo',[0.6;0.7], param, loadObj);
agents(3) = agent('Gerlad',[1.2;0.7],param, loadObj);
agents(4) = agent('Leila',[0.7;1.1], param, loadObj);
agents(1).attach();
agents(2).attach();
agents(3).attach();
agents(4).attach();

Ts = 10e-2;

my_robot_army = flock(agents, loadObj, Ts);

% check neighbour
nodes = length(agents);
for i = 1:nodes
    for j = 1:nodes
        if(i ~= j)
            agents(i).sendMessage(agents(j),strcat(agents(i).name,'-'));
        end
    end
end

figure()
grid on
hold on
my_robot_army.plot()
my_robot_army.moveAgent(1,[1,1])
my_robot_army.plot()
hold off