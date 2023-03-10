function [allData, scenario, sensors] = TP1_DSD_104580()
%TP1_DSD_104580 - Returns sensor detections
%    allData = TP1_DSD_104580 returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = TP1_DSD_104580 optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.12 (R2022a) and Automated Driving Toolbox 3.5 (R2022a).
% Generated on: 10-Nov-2022 11:56:41

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
[sensors, numSensors] = createSensors(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {}, 'PointClouds', {}, 'INSMeasurements', {});
running = true;
while running

    % Generate the target poses of all actors relative to the ego vehicle
    poses = targetPoses(egoVehicle);
    % Get the state of the ego vehicle
    actorState = state(egoVehicle);
    time  = scenario.SimulationTime;

    objectDetections = {};
    laneDetections   = [];
    ptClouds = {};
    insMeas = {};
    isValidTime = false(1, numSensors);
    isValidLaneTime = false(1, numSensors);
    isValidPointCloudTime = false(1, numSensors);
    isValidINSTime = false(1, numSensors);

    % Generate detections for each sensor
    for sensorIndex = 1:numSensors
        sensor = sensors{sensorIndex};
        % Generate the ego vehicle lane boundaries
        if isa(sensor, 'visionDetectionGenerator')
            maxLaneDetectionRange = min(500,sensor.MaxRange);
            lanes = laneBoundaries(egoVehicle, 'XDistance', linspace(-maxLaneDetectionRange, maxLaneDetectionRange, 101));
        end
        type = getDetectorOutput(sensor);
        if strcmp(type, 'Objects only')
            if isa(sensor,'ultrasonicDetectionGenerator')
                [objectDets, isValidTime(sensorIndex)] = sensor(poses, time);
                numObjects = length(objectDets);
            else
                [objectDets, numObjects, isValidTime(sensorIndex)] = sensor(poses, time);
            end
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes only')
            [laneDets, ~, isValidTime(sensorIndex)] = sensor(lanes, time);
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes and objects')
            [objectDets, numObjects, isValidTime(sensorIndex), laneDets, ~, isValidLaneTime(sensorIndex)] = sensor(poses, lanes, time);
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes with occlusion')
            [laneDets, ~, isValidLaneTime(sensorIndex)] = sensor(poses, lanes, time);
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'PointCloud')
            if sensor.HasRoadsInputPort
                rdmesh = roadMesh(egoVehicle,min(500,sensor.MaxRange));
                [ptCloud, isValidPointCloudTime(sensorIndex)] = sensor(poses, rdmesh, time);
            else
                [ptCloud, isValidPointCloudTime(sensorIndex)] = sensor(poses, time);
            end
            ptClouds = [ptClouds; ptCloud]; %#ok<AGROW>
        elseif strcmp(type, 'INSMeasurement')
            insMeasCurrent = sensor(actorState, time);
            insMeas = [insMeas; insMeasCurrent]; %#ok<AGROW>
            isValidINSTime(sensorIndex) = true;
        end
    end

    % Aggregate all detections into a structure for later use
    if any(isValidTime) || any(isValidLaneTime) || any(isValidPointCloudTime) || any(isValidINSTime)
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections}, ...
            'LaneDetections', {laneDetections}, ...
            'PointClouds',   {ptClouds}, ... %#ok<AGROW>
            'INSMeasurements',   {insMeas}); %#ok<AGROW>
    end

    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release all the sensor objects so they can be used again.
for sensorIndex = 1:numSensors
    release(sensors{sensorIndex});
end

% save('allData_104580.mat','allData');

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
    'UpdateInterval', 0.02, ...
    'SensorLocation', [3.7 0], ...
    'Yaw', 15, ...
    'MaxRange', 20, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([1500 320],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{2} = drivingRadarDataGenerator('SensorIndex', 2, ...
    'MountingLocation', [2.8 0.9 0.2], ...
    'MountingAngles', [90 0 0], ...
    'RangeLimits', [0 20], ...
    'TargetReportFormat', 'Detections', ...
    'FieldOfView', [150 5], ...
    'Profiles', profiles);
sensors{3} = drivingRadarDataGenerator('SensorIndex', 3, ...
    'MountingLocation', [2.8 -0.9 0.2], ...
    'MountingAngles', [-90 0 0], ...
    'RangeLimits', [0 20], ...
    'TargetReportFormat', 'Detections', ...
    'FieldOfView', [150 5], ...
    'Profiles', profiles);
sensors{4} = insSensor('TimeInput', true, ...
    'MountingLocation', [0.95 0 0]);
sensors{5} = visionDetectionGenerator('SensorIndex', 5, ...
    'SensorLocation', [2.8 0.9], ...
    'Yaw', 90, ...
    'MaxRange', 20, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([320 320],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{6} = visionDetectionGenerator('SensorIndex', 6, ...
    'SensorLocation', [2.8 -0.9], ...
    'Yaw', -90, ...
    'MaxRange', 20, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([320 320],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
numSensors = 6;

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [54 -20 0;
    66.2 -5.8 0;
    66.5 64.8 0;
    50.5 81.3 0;
    -27.7 80.6 0;
    -39 60.9 0;
    -38.7 -9.5 0;
    -20.7 -20.3 0;
    54 -20 0];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the barriers
barrierCenters = [75 -1.1 0;
    58.7 -23.2 0];
barrier(scenario, barrierCenters, ...
    'ClassID', 5, ...
    'Width', 0.61, ...
    'Height', 0.81, ...
    'Mesh', driving.scenario.jerseyBarrierMesh, 'PlotColor', [0.65 0.65 0.65], 'Name', 'Jersey Barrier');

barrierCenters = [-37.9 74 0;
    -51.3 44.3 0];
barrier(scenario, barrierCenters, ...
    'ClassID', 5, ...
    'Width', 0.61, ...
    'Height', 0.81, ...
    'Mesh', driving.scenario.jerseyBarrierMesh, 'PlotColor', [0.65 0.65 0.65], 'Name', 'Jersey Barrier1');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [20.7418695013332 -31.9231626883298 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [20.7418695013332 -31.9231626883298 0;
    33.7 -31 0;
    46.7 -26.8 0;
    54.2 -22.2 0;
    61 -15.6 0;
    66.2 -9.2 0;
    71.7 1.3 0;
    75.5 11.6 0;
    78.3 22 0;
    78.6 34.3 0;
    76.6 46.1 0;
    72.6 58.4 0;
    66.9 67.8 0;
    61.8 73.4 0;
    55.1 79.5 0;
    28.9 94.2 0;
    11.3 96.6 0;
    -8.9 94.2 0;
    -24.9 85.7 0;
    -35.5 71.8 0;
    -43.1 55.3 0;
    -49.3 38.6 0;
    -51.5 21.7 0;
    -49.5 3.5 0;
    -41 -10.1 0;
    -28.2 -18.7 0;
    -12.6 -25.5 0;
    2.1 -30.1 0;
    21.7 -31.9 0];
speed = [30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30];
smoothTrajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [-44.9 18.7 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian');

actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [7.69999999999999 94.2 0], ...
    'Mesh', driving.scenario.bicycleMesh, ...
    'Name', 'Bicycle');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [44.2 92.6 0], ...
    'Yaw', -30, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'Car2');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [-21.7 79.5 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'PlotColor', [0.301 0.745 0.933], ...
    'Name', 'Pedestrian1');

actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [-8.8 -23.9 0], ...
    'Yaw', -20, ...
    'Mesh', driving.scenario.bicycleMesh, ...
    'PlotColor', [0.635 0.078 0.184], ...
    'Name', 'Bicycle1');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [76.9 28.1 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'PlotColor', [0 0.447 0.741], ...
    'Name', 'Pedestrian2');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [57.4 72.9 0], ...
    'Yaw', 135, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Car3');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-41.6 50.7 0], ...
    'Yaw', 65, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.929 0.694 0.125], ...
    'Name', 'Car4');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [59.4 -0.3 0], ...
    'Yaw', -40, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'Car5');

actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [-25.7 -16.4 0], ...
    'Yaw', -30, ...
    'Mesh', driving.scenario.bicycleMesh, ...
    'PlotColor', [0.301 0.745 0.933], ...
    'Name', 'Bicycle2');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [-35.5 68.6 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'PlotColor', [0.635 0.078 0.184], ...
    'Name', 'Pedestrian3');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [71.7 49.8 0], ...
    'Yaw', 110, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0 0.447 0.741], ...
    'Name', 'Car1');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [36.4 -26.5 0], ...
    'Yaw', 20, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Car6');

function output = getDetectorOutput(sensor)

if isa(sensor, 'visionDetectionGenerator')
    output = sensor.DetectorOutput;
elseif isa(sensor, 'lidarPointCloudGenerator')
    output = 'PointCloud';
elseif isa(sensor, 'insSensor')
    output = 'INSMeasurement';
else
    output = 'Objects only';
end

