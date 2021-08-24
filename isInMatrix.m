function [flag] = isInMatrix(element,M)
% check if the element is in the matrix M
    flag = any( M == ones(size(M)) .* element );
end

