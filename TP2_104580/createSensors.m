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

