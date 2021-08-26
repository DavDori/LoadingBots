function [ids] = attachRobotEquilibrium(ids_attachable, drive_force, priority, th_at, th_dt)
    ids = [];
    if(isempty(ids_attachable) == false) % there is at least a robot that can detach
        
        % select agents that have a drive force smaller than the threshold,
        % meaning they are close to an equilibrium
        n = 0;
        ids = zeros(size(ids_attachable));
        for j = ids_attachable
            if(drive_force(j) < th_at && priority(j) < th_dt)
                n = n + 1;
                ids(n) = j;
            end
        end
        ids = ids(1:n);
    end
end

