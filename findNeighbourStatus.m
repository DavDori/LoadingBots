function status = findNeighbourStatus(NB)
    % find if the neighbours are or not attached returning an array of
    % logic values, true = attached, false = detached
    n = length(NB);
    status = zeros(1,n);
    for i = 1:n
        status(i) = NB(i).attached;
    end
end

