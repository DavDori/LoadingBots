function [alpha] = angle2D(c, error_limit)
    if(sqrt(c*c') < error_limit) % centroid very close to 0
        alpha = [];
    else
        alpha = wrapTo2Pi(atan2(c(2), c(1)));
    end
end

