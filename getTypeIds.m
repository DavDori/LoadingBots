function [ids] = getTypeIds(type, array)
% given the array of agents, extract the ids of the conneted agents
    if(strcmp(type, 'Attached')) % use only attached agents
        ids = find(array == true);
    elseif(strcmp(type, 'Detached')) % use only detached agents
        ids = find(array == false);
    elseif(strcmp(type, 'All')) % use all agents
        ids = 1:length(array);
    else
        error('Wrong type of agent selected, can choose between: All, Attached, Detached');
    end
end

