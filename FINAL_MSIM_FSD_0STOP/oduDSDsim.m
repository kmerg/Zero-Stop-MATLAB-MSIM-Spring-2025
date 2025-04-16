function [simData, egoVehicle, sensors] = oduDSDsim()
% oduDSDsim - Constructs a scenario with expanded road centers, plus sensor attachment.
%
% Outputs:
%   simData    - A struct with fields:
%                 .scenario (the drivingScenario object)
%                 .roads    (an array of road objects)
%   egoVehicle - The main ego vehicle
%   sensors    - The sensor suite (Radar, LiDAR, INS)

    [scenario, egoVehicle, allRoads] = createDrivingScenario();
    [sensors, numSensors] = createSensors(scenario);
    addSensors(scenario, sensors, egoVehicle.ActorID);

    simData.scenario = scenario;
    simData.roads    = allRoads;
end

%% Nested function: createSensors
function [sensors, numSensors] = createSensors(scenario)
    profiles = actorProfiles(scenario);

    sensors{1} = drivingRadarDataGenerator( ...
        'SensorIndex', 1, ...
        'MountingLocation', [3.7 0 0.2], ...
        'RangeLimits', [0 100], ...
        'TargetReportFormat', 'Detections', ...
        'Profiles', profiles);

    sensors{2} = lidarPointCloudGenerator( ...
        'SensorIndex', 2, ...
        'SensorLocation', [3.7 0], ...
        'MaxRange', 100, ...
        'AzimuthLimits', [-10 10], ...
        'ActorProfiles', profiles);

    sensors{3} = insSensor( ...
        'TimeInput', true, ...
        'MountingLocation', [0.95 0 0]);

    numSensors = 3;
end

%% Nested function: createDrivingScenario
function [scenario, egoVehicle, allRoads] = createDrivingScenario()
% createDrivingScenario - Constructs the driving scenario with expanded road centers.
% Each road is appended to allRoads with a unique variable name to avoid overwrites.

    scenario = drivingScenario;
    allRoads = [];

    %% Helper definitions for repeating lanes
    lsConnector = laneSpecConnector('TaperShape', 'None');

    % -------------------------------
    % SecondStreet
    % -------------------------------
    secCenters = [...
        -99.5 50.4 0;
        -85.5 51   0;
        -54   51   0;
        -33.6 50.4 0;
        -9.7  50.5 0;
        15.7  50.8 0;
        57.4  51.7 0;
        99.5  52.5 0;
        134.4 52.9 0;
        154.4 52.5 0;
        173.8 51.9 0;
        202.1 52.1 0;
        213.9 51.3 0;
        228.5 51.9 0;
        251.6 49   0];
    laneSpecs2 = repmat(lanespec(2, 'Width', 4.5), 10, 1);
    compLane2  = compositeLaneSpec(laneSpecs2, 'Connector', lsConnector);

    rSecondStreet = road(scenario, secCenters, 'Lanes', compLane2, 'Name', 'SecondStreet');
    allRoads = [allRoads, rSecondStreet];

    % -------------------------------
    % ThirdStreet
    % -------------------------------
    thrCenters = [...
        -102.3 -51.4 0;
        -34.2  -51.3 0;
        -3.5   -51.9 0;
        11.2   -51.5 0;
        39.2   -50.7 0;
        81.9   -52.5 0;
        124.6  -50.7 0;
        161.9  -48.9 0;
        189.5  -47.1 0;
        214.2  -50.7 0;
        251.6  -50.7 0];
    rThirdStreet = road(scenario, thrCenters, 'Lanes', compLane2, 'Name', 'ThirdStreet');
    allRoads = [allRoads, rThirdStreet];

    % -------------------------------
    % FourthStreet
    % -------------------------------
    fouCenters = [...
        -100.9 -149.6 0;
        -80.5  -148.4 0;
        -63.2  -147.2 0;
        -30    -150   0;
        -6.8   -151.7 0;
        12.5   -151.7 0;
        30.7   -152.2 0;
        91.3   -150.4 0;
        145.3  -150.9 0;
        175.8  -151.3 0;
        213.8  -151.3 0;
        250.3  -152.4 0];
    rFourthStreet = road(scenario, fouCenters, 'Lanes', compLane2, 'Name', 'FourthStreet');
    allRoads = [allRoads, rFourthStreet];

    % -------------------------------
    % FirstStreet
    % -------------------------------
    firCenters = [...
        -100.9 152.7 0;
        -85.4  152.9 0;
        -62.3  151.7 0;
        -34.2  152.6 0;
        -10.5  153   0;
        5.1    152.6 0;
        153.6  150.5 0;
        170.9  150.7 0;
        188.4  150.4 0;
        201.2  151.1 0;
        221    152.1 0;
        250.3  150   0];
    rFirstStreet = road(scenario, firCenters, 'Lanes', compLane2, 'Name', 'FirstStreet');
    allRoads = [allRoads, rFirstStreet];

    % -------------------------------
    % MonarchWay
    % -------------------------------
    monCenters = [...
        161.2 201.8 0;
        161.7 182.5 0;
        162.3 158.4 0;
        162.8 142.4 0;
        164.1 73.7  0;
        165   25.7  0;
        165.4 -14.3 0;
        166.1 -71.6 0;
        167.1 -121.4 0;
        167.5 -178.5 0;
        167.7 -242.9 0;
        168.7 -278.4 0;
        171.1 -308.2 0;
        171.1 -342.3 0;
        171.5 -370.8 0];
    laneSpecsM = repmat(lanespec(2,'Width',4.5),25,1);
    compLaneM  = compositeLaneSpec(laneSpecsM, 'Connector', lsConnector);

    rMonarchWay = road(scenario, monCenters, 'Lanes', compLaneM, 'Name','MonarchWay');
    allRoads = [allRoads, rMonarchWay];

    % -------------------------------
    % HamptonBLVD
    % -------------------------------
    hamCenters = [...
        -1.3  318.2  0;
        -2.4  298.4  0;
        -4.2  286.3  0;
        -4.9  264.4  0;
        -4.7  241.5  0;
        -4.9  215.8  0;
        -4.5  194    0;
        -4.2  174.7  0;
        -4    159.8  0;
        -2.9  145.5  0;
        -2.7  137.9  0;
        -2.7  129.6  0;
        -2.3  115.2  0;
        -1.9  100.3  0;
        -0.8  85.4   0;
        0.9   58.1   0;
        1.8   43     0;
        0.7   -9.8   0;
        0.7   -41.5  0;
        0.4   -59.9  0;
        -0.3  -99.8  0;
        0.7   -136.6 0;
        1     -162.5 0;
        -0.2  -202.3 0;
        0.1   -264   0;
        -0.8  -315.1 0;
        0.1   -374.6 0;
        1.7   -463.8 0];
    headings = [-90; NaN; NaN; -90; NaN; -90; NaN; NaN; NaN; ...
                NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ...
                NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN];

    laneSpecsH = repmat(lanespec(2,'Width',4.5),25,1);
    compLaneH  = compositeLaneSpec(laneSpecsH,'Connector',lsConnector);

    rHamptonBLVD = road(scenario, hamCenters, 'Heading', headings, ...
        'Lanes', compLaneH, 'Name','HamptonBLVD');
    allRoads = [allRoads, rHamptonBLVD];

    
%% Reformatted Parking Spaces and ODU

% ParkingSpace1
roadCenters = [10.1 309.3 0; -19.9 309.4 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r1 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace1');
allRoads = [allRoads, r1];

% ParkingSpace2
roadCenters = [-20.6 292.8 0; 20.7 292.1 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r2 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace2');
allRoads = [allRoads, r2];

% ParkingSpace3
roadCenters = [-95.3 146 0; -95.5 180.2 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r3 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace3');
allRoads = [allRoads, r3];

% ParkingSpace4
roadCenters = [-74.2 161.4 0; -75.4 130.8 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r4 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace4');
allRoads = [allRoads, r4];

% ParkingSpace5
roadCenters = [-92.4 39.2 0; -91.5 75.4 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r5 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace5');
allRoads = [allRoads, r5];

% ParkingSpace6
roadCenters = [-72.8 58.2 0; -74.7 50.8 0; -73 24.9 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 3, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r6 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace6');
allRoads = [allRoads, r6];

% ParkingSpace7
roadCenters = [-87.8 -59.3 0; -87.2 -23.2 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r7 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace7');
allRoads = [allRoads, r7];

% ParkingSpace8
roadCenters = [-68 -81.2 0; -67.7 -64.6 0; -67.2 -44.5 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 3, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r8 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace8');
allRoads = [allRoads, r8];

% ParkingSpace9
roadCenters = [-75.1 -137.3 0; -75.4 -176.2 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r9 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace9');
allRoads = [allRoads, r9];

% ParkingSpace10
roadCenters = [-93.7 -158.8 0; -93.6 -127.8 0; -93.6 -120.2 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 3, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r10 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace10');
allRoads = [allRoads, r10];

% ParkingSpace11
roadCenters = [191 124 0; 191.8 157.9 0; 192.1 173.6 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 3, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r11 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace11');
allRoads = [allRoads, r11];

% ParkingSpace12
roadCenters = [230.8 157.8 0; 230.4 116.8 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r12 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace12');
allRoads = [allRoads, r12];

% ParkingSpace13
roadCenters = [171.3 190.4 0; 154.8 190.8 0; 120.3 189.3 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 3, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r13 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace13');
allRoads = [allRoads, r13];

% ParkingSpace14
roadCenters = [228.6 41.1 0; 228.1 81.9 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r14 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace14');
allRoads = [allRoads, r14];

% ParkingSpace15
roadCenters = [208.1 57.8 0; 208 28.1 0; 209.3 10.3 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 3, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r15 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace15');
allRoads = [allRoads, r15];

% ParkingSpace16
roadCenters = [233.6 -41.8 0; 233.4 -90.1 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r16 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace16');
allRoads = [allRoads, r16];

% ParkingSpace17
roadCenters = [191.8 -59.6 0; 190.9 -16.7 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r17 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace17');
allRoads = [allRoads, r17];

% ParkingSpace18
roadCenters = [228.8 -143.7 0; 229.1 -185.6 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r18 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace18');
allRoads = [allRoads, r18];

% ParkingSpace19
roadCenters = [196.6 -160.7 0; 197 -116.1 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r19 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace19');
allRoads = [allRoads, r19];

% ParkingSpace20
roadCenters = [160.9 -290.1 0; 199.8 -291.2 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r20 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ParkingSpace20');
allRoads = [allRoads, r20];

% ODU
roadCenters = [-4.7 -268 0; 59 -268 0];
laneSpecs = repmat(lanespec(1, 'Width', 6), 2, 1);
compLaneSpec = compositeLaneSpec(laneSpecs, 'Connector', lsConnector);
r21 = road(scenario, roadCenters, 'Lanes', compLaneSpec, 'Name', 'ODU');
allRoads = [allRoads, r21];


    % Ego Vehicle
    egoVehicle = vehicle(scenario, ...
        'ClassID', 1, ...
        'Position', [-100.25 -53.91 0.01], ...
        'Mesh', driving.scenario.carMesh, ...
        'Name','EgoVehicle');
end
