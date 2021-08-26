function [map_updated] = addDisksToMap(map, positions, radii)
    % positions is a 2xn matrix where n is the number of objects to be
    % added
    map_updated = binaryOccupancyMap(map.XLocalLimits(2), ...
                                map.YLocalLimits(2), ...
                                map.Resolution);
    for i = 1:size(positions, 2)
        map_updated.setOccupancy(positions(:,i)', 1); 
        
    end
    map_updated.inflate(radii);
end

