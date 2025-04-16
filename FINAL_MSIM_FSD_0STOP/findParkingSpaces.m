function parkingSpaces = findParkingSpaces(simData)
% findParkingSpaces - Scans simData.roads for roads whose Name
% starts with "ParkingSpace" and returns a struct array:
%   parkingSpaces(i).Name = 'ParkingSpaceX'
%   parkingSpaces(i).Position = [x,y]
%
% The coordinate is chosen as the FINAL point in RoadCenters.

    parkingSpaces = struct('Name', {}, 'Position', {});
    
    allRoads = simData.roads;  % array built in createDrivingScenario

    % Identify roads with names that start with 'ParkingSpace'
    idx = arrayfun(@(r) startsWith(r.Name, "ParkingSpace"), allRoads);
    parkingRoads = allRoads(idx);

    % For each matching road, pick the final coordinate of RoadCenters
    for i = 1:numel(parkingRoads)
        nameStr = parkingRoads(i).Name;
        centers = parkingRoads(i).RoadCenters;
        
        % If you want the final point:
        % spotPos = centers(end, 1:2);
        % If you prefer the first point:
        % spotPos = centers(1, 1:2);
        % Or a midpoint:
        % spotPos = mean(centers(:,1:2), 1);
        
        spotPos = centers(end, 1:2);  % final point in XY

        parkingSpaces(i).Name = nameStr;
        parkingSpaces(i).Position = spotPos;
    end
end
