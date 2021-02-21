function [x, y] = polar2cartesian(rho, phi)
%POLAR2CARTESIAN change from polar coordinates to cartesian coordinates
    x = rho .* cos(phi);
    y = rho .* sin(phi);
end

