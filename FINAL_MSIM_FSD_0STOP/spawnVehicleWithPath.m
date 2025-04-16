function vehicleObj = spawnVehicleWithPath(scenario, startPos, goalPos, name)
    vehicleObj = vehicle(scenario, 'ClassID', 1, ...
               'Position', [startPos, 0], 'Name', name);

    waypoints = planPath(startPos, goalPos);
    speed     = 20 * ones(size(waypoints,1),1);
    smoothTrajectory(vehicleObj, waypoints, speed);
end
