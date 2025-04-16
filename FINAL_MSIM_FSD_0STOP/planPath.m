function waypoints = planPath(startPos, goalPos)
% planPath - Rebuilds lane graph from simData, then does A* from nearest node to nearest node

    global drivingScenarioObject;
    if isempty(drivingScenarioObject)
        error('Global scenario object not set. Must do: global drivingScenarioObject = simData;');
    end

    % 1) Generate lane-level info with user prompt for step size
    laneInfo = lanes(drivingScenarioObject);

    % 2) Build the strictly lane-following graph 
    G = buildLaneGraph(laneInfo);

    % 3) Find nearest node to startPos, goalPos
    startNode = findClosestNode(G.Nodes.Pos, startPos);
    goalNode  = findClosestNode(G.Nodes.Pos, goalPos);

    % 4) A* search (shortestpath)
    pathIndices = shortestpath(G, startNode, goalNode);

    if numel(pathIndices)<2
        warning('No multi-node path found. Using direct line fallback.');
        waypoints = [startPos; goalPos];
    else
        waypoints = G.Nodes.Pos(pathIndices,:);
        fprintf('[planPath] Found path with %d waypoints.\n', size(waypoints,1));
    end
end

function idx = findClosestNode(nodePos, xy)
    dists = vecnorm(nodePos(:,1:2) - xy(1:2),2,2);
    [~, idx] = min(dists);
end
