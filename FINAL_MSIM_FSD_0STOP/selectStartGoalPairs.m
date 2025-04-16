function pairs = selectStartGoalPairs(availableSpots, numPairs)
% selectStartGoalPairs - Randomly selects start and goal pairs
%   Ensures they are not the same and sufficiently spaced apart

pairs = struct('start', {}, 'goal', {});

idx = randperm(size(availableSpots, 1));
for i = 1:numPairs
    s = availableSpots(idx(i), :);

    % Choose goal far from start
    remaining = availableSpots;
    dists = vecnorm(remaining - s, 2, 2);
    farEnough = find(dists > 10); % 10m spacing rule

    if isempty(farEnough)
        warning('Not enough spaced parking spots. Using closest available.');
        g = remaining(1,:);
    else
        g = remaining(farEnough(randi(length(farEnough))), :);
    end

    pairs(i).start = s;
    pairs(i).goal = g;
end
end
