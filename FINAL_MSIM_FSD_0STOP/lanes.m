function laneInfo = lanes(simData)
% lanes - Generates lane-level data for each road in simData.roads.
% Prompts user for a "step size" for interpolation. Then, for each road:
%   - Interpolates the RoadCenters at ~stepSize spacing
%   - If road is 'ParkingSpaceX', uses 1 lane of width=6
%   - Otherwise uses 2 lanes of width=4.5 each
%   - Offsets lane(s) from the center line
%   - Optionally does a second smoothing pass (example code is included but disabled)
%
% Outputs a struct array laneInfo:
%   laneInfo(i).roadName       = string
%   laneInfo(i).centerPoints   = Nx3 (the dense center, stepSize spacing)
%   laneInfo(i).lanePoints{k}  = Nx3 for lane k

    if ~isfield(simData, 'roads')
        error('simData must contain .roads. Check oduDSDsim output.');
    end

    roads = simData.roads;
    % Prompt user for step size
    stepSize = input('Enter step size (meters) for primary interpolation (e.g. 10): ');

    laneInfo = struct('roadName', {}, 'centerPoints', {}, 'lanePoints', {});

    for rIndex = 1:numel(roads)
        roadObj = roads(rIndex);
        roadName = roadObj.Name;
        coarseCenters = roadObj.RoadCenters;  % Nx3

        % 1) Interpolate primary
        denseCenter = interpolateCoarseCenters(coarseCenters, stepSize);

        % 2) Decide # of lanes
        % If name starts with ParkingSpace => 1 lane, width=6
        % else 2 lanes, each 4.5 wide => total = 9
        if startsWith(roadName, "ParkingSpace", "IgnoreCase",true)
            numLanes = 1;
            laneWidth = 6;  % single-lane
        else
            numLanes = 2;
            laneWidth = 4.5; % each lane
        end
        totalWidth = numLanes * laneWidth;

        % 3) Offset each lane from center
        laneArrays = cell(1,numLanes);
        for laneIdx = 1:numLanes
            % Lane offset from the road center line
            % Example: if numLanes=2, offsets = +2.25 and -2.25
            offsetFromCenter = (laneIdx - 0.5 - numLanes/2) * laneWidth;

            laneArrays{laneIdx} = offsetLane(denseCenter, offsetFromCenter);
        end

        % Optional: second pass smoothing if you want
        % e.g. laneArrays{k} = generatePreciseWaypointsSpline(laneArrays{k}, 2*size(laneArrays{k},1));

        % Store results
        laneInfo(rIndex).roadName     = roadName;
        laneInfo(rIndex).centerPoints = denseCenter;  % Nx3
        laneInfo(rIndex).lanePoints   = laneArrays;   % 1 or 2 cells, each Nx3
    end
end

%% Helper function to interpolate coarse centers
function denseCenters = interpolateCoarseCenters(coarseCenters, stepSize)
    denseCenters = [];
    for i = 1:(size(coarseCenters,1)-1)
        p1 = coarseCenters(i, :);
        p2 = coarseCenters(i+1, :);
        segVec = p2 - p1;
        segDist = norm(segVec(1:2));
        if segDist < 1e-6
            continue; % skip degenerate
        end
        numSteps = ceil(segDist / stepSize);
        for s = 0:numSteps
            t = s / numSteps;
            newPt = p1 + t * segVec;
            denseCenters(end+1,:) = newPt; %#ok<AGROW>
        end
    end
end

%% Helper function to offset entire lane
function laneXYZ = offsetLane(denseCenter, offsetDist)
% offsetLane - for each consecutive pair of points in denseCenter,
% compute a normal, then offset by offsetDist. We'll do a simpler
% approach that just uses the local tangent. If offsetDist=+2.25 => lane
% is "left" of center, if -2.25 => right.
    laneXYZ = denseCenter;  % copy
    nPoints = size(denseCenter,1);
    if nPoints<2
        return
    end

    for i = 1:(nPoints-1)
        p1 = denseCenter(i,1:2);
        p2 = denseCenter(i+1,1:2);
        t = p2 - p1;  % tangent
        tNorm = norm(t);
        if tNorm < 1e-6
            continue
        end
        tHat = t / tNorm;
        % left normal
        nHat = [-tHat(2), tHat(1)];
        % current offset
        offsetVec = offsetDist * nHat;
        laneXYZ(i,1:2) = laneXYZ(i,1:2) + offsetVec;
    end
    % fix the last point similarly
    laneXYZ(end,1:2) = laneXYZ(end-1,1:2);  % or offset the last segment too
end