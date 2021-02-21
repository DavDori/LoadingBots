function R = rotationMatrix(theta)
    % compute the 2D rotational matrix
    R = [cos(theta), -sin(theta);...
         sin(theta), cos(theta)];
end
