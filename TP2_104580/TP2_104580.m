clc,clear,close all

addpath functions\
addpath scenarios\

% criar cenário
[~, scenario, ~] = scenario2();
[sensors, ~] = createSensors(scenario);

egoVehicle = scenario.Actors(1);
scenario.SampleTime = 0.1;

% Collision check time stamps
tSteps = 0.5:0.5:5;
maxTimeHorizon = max(tSteps);

% Create the dynamicCapsuleList object
capList = dynamicCapsuleList;
% capList.MaxNumSteps = numel(tSteps) + 1;
capList.MaxNumSteps = 1+floor(maxTimeHorizon/scenario.SampleTime);

% Specify the ego vehicle geometry
carLen = 4.7;
carWidth = 1.8;
rearAxleRatio = 0.25;
egoID = 1;
laneWidth = 3.6;
[egoID, egoGeom] = egoGeometry(capList,egoID);

% Inflate to allow uncertainty and safety gaps
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters
updateEgoGeometry(capList,egoID,egoGeom);

% Create display for visualizing results
display = TrackingAndPlanningDisplay;

% Initial state of the ego vehicle
refPath = getReferencePath();
connector = trajectoryGeneratorFrenet(refPath);
egoState = frenet2global(refPath,[0 0 0 -0.5*laneWidth 0 0]);
moveEgoToState(egoVehicle, egoState);

tracker = trackerJPDA('FilterInitializationFcn',@initRefPathFilter,...
                        'AssignmentThreshold',[200 inf],...
                        'ConfirmationThreshold',[8 10],...
                        'DeletionThreshold',[5 5]);

while advance(scenario)

    % Current time
    time = scenario.SimulationTime;

    % Obter as deteções
    detections = helperGenerateDetections(sensors, egoVehicle, time);    
    tracks = tracker(detections, time);

    timesteps = time + tSteps;
    predictedTracks = repmat(tracks,[1 numel(timesteps)+1]);
    for i = 1:numel(timesteps)
        predictedTracks(:,i+1) = tracker.predictTracksToTime('confirmed',timesteps(i));
    end

    % Atualizar as cápsulas dos outros atores
    currActorState = updateCapsuleList(capList, predictedTracks, @stateToPoseFrenet);

    % Planear a trajetória
    [optimalTrajectory, trajectoryList] = planTrajectory(capList, currActorState, egoState);

    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);

    % Mover o veiculo para a posição atual
    if ~isempty(optimalTrajectory)
        last_optimalTrajectory = optimalTrajectory;
        egoState = optimalTrajectory(2,:);   
        moveEgoToState(egoVehicle,egoState);
    else
        optimalTrajectory = last_optimalTrajectory;
        egoState = optimalTrajectory(2,:);   
        moveEgoToState(egoVehicle,egoState);
    end

end