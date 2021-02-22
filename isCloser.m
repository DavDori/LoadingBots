function [r] = isCloser(neighbours, point, agent_size)
    % check if the obj agent is the closest to a point, 1 it's the
    % closest, 0 it's not. Coordinates are considered in the
    % reference frame of the obj agent.
    r = 1;
    if(isempty(neighbours) == false)
        
        dist_obj = sqrt(point(1)^2 + point(2)^2);
        delta_agents_size = 2 * agent_size;
        
        for i = 1:size(neighbours, 1)
            % consider collision avoidance between agents
            agents_dist = sqrt(neighbours(i,1)^2 + neighbours(i,2)^2);
            if(delta_agents_size > agents_dist / 2)
                factor = 2 * (delta_agents_size - agents_dist / 2) / agents_dist;
                neighbour_pos = neighbours(i,:) * factor;
            else
                neighbour_pos = neighbours(i,:);
            end
            neighbour_dist = sqrt((point(1) - neighbour_pos(1))^2 + ...
                                 (point(2) - neighbour_pos(2))^2);
            if(neighbour_dist < dist_obj)
                r = 0;
                break;
            end
        end
    end
end

