function availableSpots = excludeUsedSpots(allSpots, usedSpots)
% excludeUsedSpots - Removes used positions from parking list
%   Returns array of [x, y] positions not overlapping with used ones

threshold = 1.0; % meters tolerance
availableSpots = [];

for i = 1:size(allSpots, 1)
    dists = vecnorm(usedSpots - allSpots(i,:), 2, 2);
    if all(dists > threshold)
        availableSpots(end+1,:) = allSpots(i,:); %#ok<AGROW>
    end
end
end
