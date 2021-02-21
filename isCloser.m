function [r] = isCloser(neighbours, point)
    % check if the obj agent is the closest to a point, 1 it's the
    % closest, 0 it's not. Coordinates are conidered in the
    % reference frame of the obj agent.
    if(isempty(neighbours) == true)
        r = 1;
    else
        dist_obj = sqrt(point(1)^2 + point(2)^2);
        r = 1;
        for i = 1:size(neighbours, 1)
            neighbor_dist = sqrt((point(1) - neighbours(i,1))^2 + ...
                                 (point(2) - neighbours(i,2))^2);
            if(neighbor_dist < dist_obj)
                r = 0;
                break;
            end
        end
    end
end

