function [map] = png2BOMap(path, resolution)
    map = imread(path);
    map = rgb2gray(map);
    map = map < 0.5;
    map = binaryOccupancyMap(map, resolution);
    map = binaryOccupancyMap(map);
end

