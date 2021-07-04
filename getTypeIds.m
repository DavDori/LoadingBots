function [ids] = getTypeIds(type, NB)
% given the array of agents, extract the ids of the conneted agents
    status = findNeighbourStatus(NB);
    if(strcmp(type, 'Attached')) % use only attached agents
        ids = find(status == true);
    elseif(strcmp(type, 'Detached')) % use only detached agents
        ids = find(status == false);
    elseif(strcmp(type, 'All')) % use all agents
        ids = 1:length(NB);
    else
        error('Wrong type of agent selected, can choose between: All, Attached, Detached');
    end
end

