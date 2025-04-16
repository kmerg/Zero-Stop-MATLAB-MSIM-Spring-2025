function runSim()
    % runSim  
    %
    %  still rely on:
    %   - findParkingSpaces(simData) to gather "ParkingSpace" roads
    %   - excludeUsedSpots, selectStartGoalPairs for multi-vehicle logic
    %   - planPath -> which calls lanes.m, buildLaneGraph.m for lane-based navigation

    %% === 1. Load Scenario & Sensors === %%
    [simData, egoVehicle, sensors] = oduDSDsim();
    global drivingScenarioObject;
    drivingScenarioObject = simData;  % simData contains scenario + roads
    scenario = simData.scenario;

    %% === 2. Gather Parking Spaces from Lane Info === %%
    % We call findParkingSpaces to pick the final coordinate from each
    % "ParkingSpaceX" road, letting the user choose a named spot.
    parkingSpaces = findParkingSpaces(simData);
    allSpots = arrayfun(@(p) p.Position, parkingSpaces, 'UniformOutput', false);
    allSpots = vertcat(allSpots{:});
    spotNames = arrayfun(@(p) p.Name, parkingSpaces, 'UniformOutput', false);

    %% === 3. Prompt for Ego Start & Destination === %%
    fprintf('\nAvailable Parking Spots:\n');
    for i = 1:length(spotNames)
        fprintf('%2d: %s\n', i, spotNames{i});
    end
    startChoice = input('Select a parking space for the Ego START (1-20): ');
    goalChoice  = input('Select a parking space for the Ego DESTINATION (1-20): ');

    egoStart = allSpots(startChoice, :);
    egoGoal  = allSpots(goalChoice, :);

    % Move the ego to the start position
    egoVehicle.Position = [egoStart(1), egoStart(2), 0];

    %% === 4. Plan Ego Vehicle Path (Lane-Based) === %%
    % planPath now calls lanes(simData), buildLaneGraph, etc.
    egoPath = planPath(egoStart, egoGoal);
    speed   = 30 * ones(size(egoPath,1),1);
    smoothTrajectory(egoVehicle, egoPath, speed);

    %% === 5. Prompt for # of Other Vehicles === %%
    numOtherVehicles = input('Enter number of other vehicles to generate: ');

    %% === 6. Multi-Vehicle Start/Goal Pairing === %%
    usedSpots = [egoStart; egoGoal];
    availableSpots = excludeUsedSpots(allSpots, usedSpots);
    pairs = selectStartGoalPairs(availableSpots, numOtherVehicles);

    %% === 7. Spawn Other Vehicles Using planPath as Well === %%
    otherVehPaths = cell(numOtherVehicles,1);
    for i = 1:numOtherVehicles
        name = sprintf('Vehicle%d', i);
        startPos = pairs(i).start;
        goalPos  = pairs(i).goal;

        % Lane-based path for each vehicle:
        path_i = planPath(startPos, goalPos);
        speed_i = 20 * ones(size(path_i,1),1);

        vehicleObj = vehicle(scenario, 'ClassID', 1, ...
            'Position', [startPos(1), startPos(2), 0], 'Name', name);
        smoothTrajectory(vehicleObj, path_i, speed_i);

        otherVehPaths{i} = path_i;
    end

    %% === 8. Zero-Stop Check (Collision Avoidance Placeholder) === %%
    zeroStop(scenario);

    %% === 9. Simulate and Log === %%
    allData = struct('Time', {}, 'ActorPoses', {});
    while advance(scenario)
        allData(end+1).Time = scenario.SimulationTime; %#ok<AGROW>
        allData(end).ActorPoses = actorPoses(scenario); %#ok<AGROW>
    end

    %% === 10. Visualize the Lane Info + Paths === %%
    % We'll call lanes(simData) again to retrieve the lane-based data
    laneInfo = lanes(simData);

    figure('Name','Road and Path Visualization');
    hold on; grid on; axis equal;
    xlabel('X(m)'); ylabel('Y(m)');
    title('Lane-Based Roads and Planned Paths');

    % For each road, we might have 1 or 2 lanes (or more).
    % "lanePoints" is a cell array. We can plot them all.
    for r = 1:numel(laneInfo)
        roadName = laneInfo(r).roadName;
        for L = 1:numel(laneInfo(r).lanePoints)
            laneXYZ = laneInfo(r).lanePoints{L};
            plot(laneXYZ(:,1), laneXYZ(:,2), '.-');
            hold on;
        end
    end

    % Plot the Ego's path in red
    plot(egoPath(:,1), egoPath(:,2), 'r-o', 'LineWidth',2, ...
         'DisplayName','Ego Path');

    % Plot each other vehicle's path in different colors
    colors = {'g','b','m','c','k'};
    for i = 1:numOtherVehicles
        path_i = otherVehPaths{i};
        cc = colors{mod(i-1, length(colors))+1};
        plot(path_i(:,1), path_i(:,2), [cc,'-o'], 'LineWidth',2, ...
             'DisplayName', sprintf('Veh%d Path', i));
    end

    legend('Location','bestoutside');
    disp('Simulation complete.');
end
