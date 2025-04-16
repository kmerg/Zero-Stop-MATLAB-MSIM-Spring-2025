function G = buildNavigationGraph(laneInfo, threshold)
% buildNavigationGraph - Builds a navigation graph from dense lane points.
%
% Syntax: G = buildNavigationGraph(laneInfo, threshold)
%
% Input:
%   laneInfo  - A structure array produced by lanes.m. Each element has a
%               'center' field (an N×3 matrix of waypoints).
%   threshold - Distance threshold (in meters) to connect nodes from different roads.
%               Default is 10.
%
% Output:
%   G - A digraph where nodes are the dense lane points and edges connect:
%         (a) consecutive points (intra-road connectivity) and
%         (b) selected candidate nodes (endpoints and mid-road samples) 
%             from different roads if they lie within the threshold.
%
% This version includes high levels of debugging output and a 2-D plot
% of the generated navigation graph.

debug = true;
if nargin < 2 || isempty(threshold)
    threshold = 10;  % Default threshold
end

if debug
    fprintf('--- Building Navigation Graph ---\n');
    fprintf('Using threshold = %d m\n', threshold);
end

allNodes = [];
edges = [];
laneNodeIndices = cell(numel(laneInfo),1);
nodeCount = 0;
roadIDs = [];  % Store road id for each node

% Build intra-road connectivity: add every node along each road.
for i = 1:numel(laneInfo)
    lanePoints = laneInfo(i).center;  % e.g., 300x3 points
    n = size(lanePoints,1);
    indices = (nodeCount+1):(nodeCount+n);
    laneNodeIndices{i} = indices;
    allNodes = [allNodes; lanePoints];
    roadIDs = [roadIDs; i*ones(n,1)];  % Assign road id
    newRoadEdges = [indices(1:end-1)', indices(2:end)'];
    edges = [edges; newRoadEdges];
    nodeCount = nodeCount + n;
    if debug
        fprintf('Road %d: Added %d nodes (indices %d to %d).\n', i, n, indices(1), indices(end));
    end
end

% Create the initial graph with intra-road connectivity.
G = digraph(edges(:,1), edges(:,2));
G.Nodes.Pos = allNodes;

if debug
    fprintf('Total nodes (intra-road): %d\n', size(allNodes,1));
    fprintf('Total intra-road edges: %d\n', size(edges,1));
end

%% Add Inter-road Connectivity Using Candidate Nodes
% Instead of only endpoints, sample candidate nodes (including mid-road points)
% from each road. Here we sample up to 5 nodes per road (or fewer if the road has less nodes).

numSamples = 5;  % Number of candidate nodes per road
candidateIndices = [];
candidateRoadIDs = [];
for i = 1:numel(laneInfo)
    indices = laneNodeIndices{i};
    n = length(indices);
    sampleCount = min(numSamples, n);
    sampleIdx = unique(round(linspace(1, n, sampleCount)));
    candidateIndices = [candidateIndices; indices(sampleIdx)'];
    candidateRoadIDs = [candidateRoadIDs; i*ones(length(sampleIdx), 1)];
    if debug
  %      fprintf('Road %d: Selected %d candidate nodes (from %d total nodes).\n', i, length(sampleIdx), n);
    end
end

candidateNodes = allNodes(candidateIndices, :);
if debug
    fprintf('Total candidate nodes for inter-road connectivity: %d\n', size(candidateNodes, 1));
end

% Compute pairwise distances between candidate nodes.
D = pdist2(candidateNodes, candidateNodes);
[row, col] = find(D < threshold & D > 0);

newEdges = [];
for k = 1:length(row)
    % Only connect nodes from different roads.
    if candidateRoadIDs(row(k)) ~= candidateRoadIDs(col(k))
        if row(k) < col(k)  % Avoid duplicate edges
            newEdges = [newEdges; candidateIndices(row(k)), candidateIndices(col(k));
                                   candidateIndices(col(k)), candidateIndices(row(k))]; %#ok<AGROW>
        end
    end
end

if ~isempty(newEdges)
    if debug
        fprintf('Adding %d inter-road connectivity edges (using candidate nodes, threshold = %d m).\n', size(newEdges,1), threshold);
    end
    G = addedge(G, newEdges(:,1), newEdges(:,2));
else
    if debug
        fprintf('No inter-road connectivity edges found among candidate nodes with threshold = %d m.\n', threshold);
    end
end

if debug
    fprintf('Total edges after inter-road connectivity: %d\n', numedges(G));
end

%% 2-D Plot of the Navigation Graph
if debug
    figure;
    % Plot all nodes and edges.
    h = plot(G, 'XData', G.Nodes.Pos(:,1), 'YData', G.Nodes.Pos(:,2), 'MarkerSize', 3, 'NodeColor', 'b');
    title('Navigation Graph');
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on;
    hold on;
    % Overlay candidate nodes in red for clarity.
    plot(candidateNodes(:,1), candidateNodes(:,2), 'ro', 'MarkerSize', 6, 'LineWidth', 2);
    legend(h, 'Graph Edges', 'Candidate Nodes');
    hold off;
    fprintf('Navigation graph plotted.\n');
end

if debug
    fprintf('--- Navigation Graph Construction Complete ---\n');
end

end
