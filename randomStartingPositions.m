function [pos] = randomStartingPositions(rect_dim, divisions, n_agents)
% it discretizes a given area and assign positions for a given number of
% agnents randomically
    pos = zeros(2, n_agents);
    dim_starting_spot = rect_dim / divisions;
    spots = 0:(divisions^2)-1; % all possible positions starting from zero
    p = zeros(2,n_agents);
    for i = 1:n_agents
        index = ceil(rand(1) * length(spots));
        
        p(:,i) = [floor(spots(index) / divisions); rem(spots(index), divisions)];
        % limit the spawing area into a subarea of the spot
        offset = dim_starting_spot/3 + dim_starting_spot/3 .* rand(2,1);
        pos(:,i) = p(:,i) .* dim_starting_spot + offset;
        spots(index) = []; % the chance to access the same spot is denied
    end 
end

