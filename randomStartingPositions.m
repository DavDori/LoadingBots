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
        pos(:,i) = p(:,i) .* dim_starting_spot + dim_starting_spot / 2;
        spots(index) = []; % the chance to access the same spot is denied
    end 
end

