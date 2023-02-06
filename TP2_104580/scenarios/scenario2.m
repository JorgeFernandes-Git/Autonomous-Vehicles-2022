function [allData, scenario, sensors] = scenario2()
%scenario2 - Returns sensor detections
%    allData = scenario2 returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = scenario2 optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.12 (R2022a) and Automated Driving Toolbox 3.5 (R2022a).
% Generated on: 17-Dec-2022 23:19:05

% Create the drivingScenario object and ego car
[scenario, ~] = createDrivingScenario;

% Create all the sensors
[sensors, ~] = createSensors(scenario);

allData = {};

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function [sensors, numSensors] = createSensors(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors{1} = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [1.9 0], ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([1000 1000],[1000 1000],[2000 2000]), ...
    'ActorProfiles', profiles);
sensors{2} = visionDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [0 0], ...
    'Yaw', -180, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([1000 1000],[1000 1000],[2000 2000]), ...
    'ActorProfiles', profiles);
numSensors = 2;

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 30);

% Add all road segments
roadCenters = [-14.4 53.1 0;
    4.52 47.02 0;
    23.4 37.4 0;
    23.5 7.8 0;
    25.1 -27.6 0;
    42 -40.2 0;
    61.4 -39.2 0;
    71.8 -24 0;
    98.7 -20.5 0];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-18.78 55.76 0], ...
    'Yaw', -34, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-2.97 47.51 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [-2.97 47.51 0;
    23.73 34.13 0.01;
    24.87 29.68 0.01;
    25.04 23.21 0.01;
    24.26 18.41 0.01;
    23.12 13.6 0.01;
    21.9 9.76 0.01;
    20.24 2.86 0.01;
    14.82 -1.34 0;
    10.89 -1.42 0];
speed = [8;8;8;8;8;8;8;8;8;8];
yaw =  [-18;NaN;NaN;NaN;NaN;NaN;NaN;NaN;NaN;NaN];
trajectory(car1, waypoints, speed, 'Yaw', yaw);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [29.69 -30.29 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
waypoints = [29.69 -30.29 0.01;
    26.37 -26.29 0.01;
    24.47 -21.54 0.01;
    22.92 -14.77 0.01;
    22.69 -5.45 0.01;
    23.99 2.92 0.01;
    26.1 9.4 0;
    28.5 17 0;
    29.2 26 0;
    28.2 34.8 0;
    32 41.3 0;
    39.1 37.8 0];
speed = [8;8;8;8;8;8;8;8;8;8;8;8];
trajectory(car2, waypoints, speed);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [33 -37.92 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');
waypoints = [33 -37.92 0.01;
    40.46 -41.65 0.01;
    47.25 -43.47 0.01;
    54.52 -43.66 0.01;
    61.03 -41.94 0.01;
    68 -35.6 0;
    70.9 -29.4 0;
    78.4 -21.8 0;
    89.3 -20.3 0];
speed = [5;5;5;5;5;5;5;5;5];
trajectory(car3, waypoints, speed);

car4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-20.3600719359437 -12.9824295976996 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car4');
waypoints = [-20.3600719359437 -12.9824295976996 0;
    13.02 -12.14 0;
    17.94 -11.64 0.01;
    23.57 -11.35 0.01;
    30.6 -9.9 0];
speed = [5;5;5;5;5];
trajectory(car4, waypoints, speed);

car5 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [73.1 4.6 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car5');
waypoints = [73.1 4.6 0;
    72.95 -4.59 0.01;
    72.01 -12.43 0.01;
    71 -20.6 0;
    67.9 -28.2 0;
    64.1 -33.9 0;
    58.7 -38.6 0;
    52.3 -39.6 0;
    43.1 -38.6 0;
    33.3 -33.2 0;
    34.3 -18.8 0;
    38.6 -8 0];
speed = [8;8;8;8;8;8;8;8;8;8;8;8];
trajectory(car5, waypoints, speed);

pedestrian = actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [21.1243624790001 -28.8878425609055 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian');
waypoints = [21.1243624790001 -28.8878425609055 0;
    27.86 -22.73 0];
speed = [1;1];
trajectory(pedestrian, waypoints, speed);

car6 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-14.6058775648532 61.0826782146543 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car6');
waypoints = [-14.6058775648532 61.0826782146543 0;
    -5.17 52.19 0.01;
    0.8 50.15 0.01;
    7.58 48.26 0.01;
    15.9 45.5 0;
    21.7 50.3 0];
speed = [5;5;5;5;5;5];
trajectory(car6, waypoints, speed);

