function [id] = detachRobotPriority(ids_detachable, priority, th_dt)
    id = [];
    if(isempty(ids_detachable) == false) % there is at least a robot that can detach
        % select the one with higher priority. One agent at the time can
        % detach
        [val_detach, id_rel] = max(priority(ids_detachable));
        id_abs = ids_detachable(id_rel); % get the absolute id of the agent
        
        if(val_detach > th_dt)
            id = id_abs;
        end
    end
end

