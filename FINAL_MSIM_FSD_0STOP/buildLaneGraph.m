function G = buildLaneGraph(laneInfo)
% buildLaneGraph - Builds a strictly lane-following graph from laneInfo
% (no big threshold or candidate sampling).
%
% Logic:
%   - Each lane array is a chain of points. We connect consecutive points.
%   - If two roads physically intersect (or come within 1-2 m?), we connect them at that intersection point.
%   - No random mid-lane bridging from large threshold.

% We'll store each lane's points in a big array, then add edges for consecutive nodes.

    allXYZ = [];       % will hold all lane coordinates ( Nx3 )
    roadIndexOfNode = []; % track which road, which lane
    laneIndexOfNode = [];
    nodeStartIdx = 1;

    % 1) Convert lane arrays into big node list
    for r = 1:numel(laneInfo)
        roadName  = laneInfo(r).roadName;
        laneCells = laneInfo(r).lanePoints;  % e.g. 1 or 2 lanes
        for L = 1:numel(laneCells)
            thesePoints = laneCells{L};  % Nx3
            nThis = size(thesePoints,1);
            idxRange = nodeStartIdx:(nodeStartIdx + nThis - 1);

            allXYZ(idxRange,:)        = thesePoints;      %#ok<AGROW>
            roadIndexOfNode(idxRange) = r;                %#ok<AGROW>
            laneIndexOfNode(idxRange) = L;                %#ok<AGROW>

            nodeStartIdx = nodeStartIdx + nThis;
        end
    end

    % Create a digraph (or simple adjacency list first)
    G = digraph();
    G = addnode(G, size(allXYZ,1));
    G.Nodes.Pos = allXYZ;  % store XY(Z) in .Pos if you like

    % 2) Intra-lane edges: consecutive points
    % We need to iterate again, but track running index
    runningIndex = 0;
    for r = 1:numel(laneInfo)
        laneCells = laneInfo(r).lanePoints;
        for L = 1:numel(laneCells)
            thesePoints = laneCells{L};
            nThis = size(thesePoints,1);
            baseIdx = runningIndex+1;  % first node index in G
            for i = 1:(nThis-1)
                src = baseIdx + i -1;
                dst = baseIdx + i; 
                G = addedge(G, src, dst);
                G = addedge(G, dst, src); % if you want bidirectional
            end
            runningIndex = runningIndex + nThis;
        end
    end

    % 3) Inter-road intersections: if two roads physically meet, connect nodes 
    % We'll do a small threshold (like 1.0 m) for connecting them.
    thresholdIntersect = 1.0;  % only connect if truly near
    % Compare all pairs of roads. For bigger scenarios, a kd-tree might help.

    nodeCount = numnodes(G);
    positions = allXYZ;  % Nx3
    for i = 1:nodeCount
        for j = (i+1):nodeCount
            % skip if same road
            if roadIndexOfNode(i)==roadIndexOfNode(j)
                continue
            end
            dist2D = norm(positions(i,1:2) - positions(j,1:2));
            if dist2D < thresholdIntersect
                G = addedge(G, i, j);
                G = addedge(G, j, i);
            end
        end
    end
end
