function dist = distance2D(point1, point2)
    % compute the distance between two points
    q = point1 - point2;
    dist = sqrt(q'*q);
end