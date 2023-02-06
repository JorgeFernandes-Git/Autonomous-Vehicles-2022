function [optimalTrajectory, trajectoryList] = planTrajectory(capList, currActorState, egoState)

% Define terminal state parameters
% Cached the reference path as a persistent variable 
% to avoid recomputing it every time the function is called.
persistent refPath
if isempty(refPath)
    refPath = getReferencePath();
end
laneWidth = 3.6;
speedLimit = 15;
timeHorizons = 1:5;
collisionCheckResolution = 5;
safetyGap = 10;
timResolution = 0.1;

% Define cost parameters.
LaneWeight = 3;
timeWeight = -1;
speedWeight =  1;

% Define kinematic parameters
maxAcceleration = 15; % m/s^2
maxCurvature = 0.5; % 1/m or rad/m
minVelocity = 0; % m/s

% Define connector as persistent variable
persistent connector
if isempty(connector)
    connector = trajectoryGeneratorFrenet(refPath, 'TimeResolution', timResolution);
end

% Generate cruise control states.
[terminalStatesCC, timesCC] = basicCruiseControl(refPath, laneWidth, egoState, speedLimit, timeHorizons);

% Generate lane change states.
[terminalStatesLC, timesLC] = basicLaneChange(refPath, laneWidth, egoState, timeHorizons);

% Generate vehicle following states.
if ~isempty(currActorState)
    [terminalStatesLVF, timesLVF] = basicLeadVehicleFollow(refPath, laneWidth, safetyGap, egoState, currActorState, timeHorizons);
else
    terminalStatesLVF = zeros(0,6);
    timesLVF = zeros(0,1);
end

% Combine the terminal states and times.
allTS = [terminalStatesCC; terminalStatesLC; terminalStatesLVF];
allDT = [timesCC; timesLC; timesLVF];
costTS = evaluateTSCost(allTS, allDT, laneWidth, speedLimit, speedWeight, LaneWeight, timeWeight);

egoFrenetState = global2frenet(refPath, egoState);
[~, globalTraj] = connect(connector, egoFrenetState, allTS, allDT);

% Determine evaluation order.
% Sorted the cost of each terminal state in ascending order 
% to iterate over the least costly terminal states first.
[~, idx] = sort(costTS);
optimalTrajectory = [];

% Eliminate trajectories that violate constraints.
isValid = evaluateTrajectory(globalTraj, maxAcceleration, maxCurvature, minVelocity);

trajectoryEvaluation = nan(numel(isValid),1);

% Check each trajectory for collisions starting with least cost.
for i = 1:numel(idx)
    if isValid(idx(i))

        % Update capsule list with the ego object's candidate trajectory.
        egoPoses.States = globalTraj(idx(i)).Trajectory(1:collisionCheckResolution:end,1:3);
        updateEgoPose(capList, 1, egoPoses);

        % Check for collisions.
        isColliding = checkCollision(capList);
        if all(~isColliding)
            % If no collisions are found, this is the optimal.
            % trajectory.
            trajectoryEvaluation(idx(i)) = 1;
            optimalTrajectory = globalTraj(idx(i)).Trajectory;
            break;
        else
            trajectoryEvaluation(idx(i)) = 0;
        end
    end
end

if isempty(optimalTrajectory)
    disp('Nenhuma trajetória possível, vai ser aplicada uma trajetória de paragem!')
    % Generate stopage states.
    [termStatesST,timesST] = basicStopTrajectory(refPath, laneWidth, egoState, timeHorizons);
    costTS = evaluateTSCost(termStatesST, timesST, laneWidth, speedLimit, speedWeight, LaneWeight, timeWeight);
    
    egoFrenetState = global2frenet(refPath, egoState);
    [~,globalTraj] = connect(connector, egoFrenetState, termStatesST, timesST);
    
    % Eliminate trajectories that violate constraints.
    isValid = evaluateTrajectory(globalTraj, maxAcceleration, maxCurvature, minVelocity);
    
    % Determine evaluation order.
    [~, idx] = sort(costTS);
    optimalTrajectory = [];
    
    trajectoryEvaluation = nan(numel(isValid),1);
    
    % Check each trajectory for collisions starting with least cost.
    for i = 1:numel(idx)
        if isValid(idx(i))
            % Update capsule list with the ego object's candidate trajectory.
            egoPoses.States = globalTraj(idx(i)).Trajectory(1:collisionCheckResolution:end,1:3);
            updateEgoPose(capList, 1,egoPoses);
    
            % Check for collisions.
            isColliding = checkCollision(capList);    
            if all(~isColliding)
                % If no collisions are found, this is the optimal.
                % trajectory.
                trajectoryEvaluation(idx(i)) = 1;
                optimalTrajectory = globalTraj(idx(i)).Trajectory;
                break;
            else
                trajectoryEvaluation(idx(i)) = 0;
            end
        end
    end
end

trajectoryList = globalTraj;

for i = 1:numel(trajectoryList)
    trajectoryList(i).Evaluation = trajectoryEvaluation(i);
    trajectoryList(i).IsValid = isValid(i);
end

end % end main funtions
%%%%%%%%%%%%%%%%%%%%%%%%%%%% functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% basicCruiseControl %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [terminalStates, times] = basicCruiseControl(refPath, laneWidth, egoState, targetVelocity, dt)
% Convert ego state to Frenet coordinates
frenetState = global2frenet(refPath, egoState);

% Determine current and future lanes
futureLane = predictLane(frenetState, laneWidth, dt);

% Convert future lanes to lateral offsets
lateralOffsets = (1-futureLane+.5)*laneWidth;

% Return terminal states
terminalStates = zeros(numel(dt),6);
terminalStates(:,1) = nan;
terminalStates(:,2) = targetVelocity;
terminalStates(:,4) = lateralOffsets;
times = dt(:);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% basicLaneChange %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [terminalStates, times] = basicLaneChange(refPath, laneWidth, egoState, dt)
if egoState(5) == 0
    terminalStates = [];
    times = [];
else
    % Convert ego state to Frenet coordinates
    frenetState = global2frenet(refPath, egoState);

    % Get current lane
    curLane = predictLane(frenetState, laneWidth, 0);

    % Determine if future lanes are available
    adjacentLanes = curLane+[-1 1];
    validLanes = adjacentLanes > 0 & adjacentLanes <= 4;

    % Calculate lateral deviation for adjacent lanes
    lateralOffset = (1-adjacentLanes(validLanes)+.5)*laneWidth;
    numLane = nnz(validLanes);

    % Calculate terminal states
    terminalStates = zeros(numLane*numel(dt),6);
    terminalStates(:,1) = nan;
    terminalStates(:,2) = egoState(5);
    terminalStates(:,4) = repelem(lateralOffset(:),numel(dt),1);
    times = repmat(dt(:),numLane,1);
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% basicLeadVehicleFollow %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [terminalStates, times] = basicLeadVehicleFollow(refPath, laneWidth, safetyGap, egoState, actorState, dt)
% Convert ego state to Frenet coordinates
frenetStateEgo = global2frenet(refPath, egoState);

% Get current lane of ego vehicle
curEgoLane = predictLane(frenetStateEgo, laneWidth, 0);

% Get current and predicted lanes for each actor
frenetStateActors = global2frenet(refPath, actorState);

predictedActorLanes = zeros(numel(dt),size(actorState,1));
for i = 1:size(actorState,1)
    predictedActorLanes(:,i) = predictLane(frenetStateActors(i,:),laneWidth,dt);
end
% For each time horizon, find the closest car in the same lane as
% ego vehicle
terminalStates = zeros(numel(dt),6);
validTS = false(numel(dt),1);
for i = 1:numel(dt)
    % Find vehicles in same lane t seconds into the future
    laneMatch = curEgoLane == predictedActorLanes(i,:)';

    % Determine if they are ahead of the ego vehicle
    leadVehicle = frenetStateEgo(1) < frenetStateActors(:,1);

    % Of these, find the vehicle closest to the ego vehicle (assume
    % constant longitudinal velocity)
    future_S = frenetStateActors(:,1) + frenetStateActors(:,2)*dt(i);
    future_S(~leadVehicle | ~laneMatch) = inf;
    [actor_S1, idx] = min(future_S);

    % Check if any car meets the conditions
    if actor_S1 ~= inf
        % If distance is greater than safety gap, set the terminal
        % state behind this lead vehicle
        if frenetStateEgo(1)+safetyGap < actor_S1
            ego_S1 = actor_S1-safetyGap;
            terminalStates(i,:) = [ego_S1 frenetStateActors(idx,2) 0 frenetStateActors(idx,4:5) 0];
            validTS(i) = true;
        end
    end
end
% Remove any bad terminal states
terminalStates(~validTS,:) = [];
times = dt(validTS(:));
times = times(:);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% evaluateTSCost %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function costs = evaluateTSCost(terminalStates, times, laneWidth, speedLimit, speedWeight, latWeight, timeWeight)
% Find lateral deviation from nearest lane center
laneCenters = (1.5:-1:-1.5)*laneWidth;
latDeviation = abs(laneCenters-terminalStates(:,4));

% Calculate lateral deviation cost
latCost = min(latDeviation,[],2)*latWeight;

% Calculate trajectory time costevaluateTrajectory
timeCost = times*timeWeight;

% Calculate terminal speed vs desired speed cost
speedCost = abs(terminalStates(:,2)-speedLimit)*speedWeight;

% Calculate if on right lane
LaneCost = abs(laneCenters(3)-terminalStates(:,4));
leftCost = min(LaneCost,[],2)*latWeight;

% Return cumulative cost
costs = latCost+timeCost+speedCost+leftCost;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% evaluateTrajectory %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function isValid = evaluateTrajectory(globalTrajectory, maxAcceleration, maxCurvature, minVelocity)
isValid = true(numel(globalTrajectory),1);
for i = 1:numel(globalTrajectory)
    % Acceleration constraint
    accViolated  = any(abs(globalTrajectory(i).Trajectory(:,6)) > abs(maxAcceleration));

    % Curvature constraint
    curvViolated = any(abs(globalTrajectory(i).Trajectory(:,4)) > abs(maxCurvature));

    % Velocity constraint
    velViolated  = any(globalTrajectory(i).Trajectory(:,5) < minVelocity);

    isValid(i) = ~accViolated && ~curvViolated && ~velViolated;
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% basicStopTrajectory %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [terminalStates, times] = basicStopTrajectory(refPath, laneWidth, egoState, dt)
if egoState(5) == 0
    terminalStates = [];
    times = [];
else
    % Convert ego state to Frenet coordinates
    frenetState = global2frenet(refPath, egoState);

    % Determine current and future lanes
    futureLane = predictLane(frenetState, laneWidth, dt);

    % Convert future lanes to lateral offsets
    lateralOffsets = (1-futureLane+.5)*laneWidth;

    % Return terminal states
    terminalStates = zeros(numel(dt),6);
    terminalStates(:,1) = nan;
    terminalStates(:,2) = 0;
    terminalStates(:,4) = lateralOffsets;
    times = dt(:);
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% predictLane %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function laneNum = predictLane(frenetState, laneWidth, dt)
narginchk(3,3);

laneBounds = [inf (1:-1:-1)*laneWidth -inf];
laneNum = zeros(numel(dt),1);

for i = 1:numel(dt)
    if dt(i) == 0
        dLaneEgo = laneBounds-frenetState(4);
        try
            laneNum(i) = find(dLaneEgo(2:(end-1)) >= 0 & dLaneEgo(3:(end)) < 0,1);
        catch
            laneNum(i) = 2;
        end
    else
        % Retrieve current velocity/acceleration/time
        t  = dt(i);
        a0 = frenetState(6);
        v0 = frenetState(5);

        % Solve for the constant change in acceleration and time of
        % application that arrest the ego vehicle's lateral
        % velocity and acceleration over a given number of seconds.
        if a0 == 0
            avgAcc = -v0/t;
            Ldiff = v0*t + avgAcc/2*t^2;
        else
            a = a0;
            b = (-2*v0-2*a0*t);
            c = (v0*t+a0/2*t^2);

            % Possible time switches
            r = (-b+(sqrt(b^2-4*a*c).*[-1 1]))/(2*a);

            % Select the option that occurs in the future
            rS = r(r>0 & r<=t);

            % Calculate the constant change in acceleration
            if ~isempty(rS)
                da0 = a0/(t-2*rS);
            else
                rS = 1;
                da0 = a0/(t-2*rS);
            end

            % Predict total distance traveled over t seconds
            Ldiff = v0*t + a0/2*t^2 + da0/6*t^3 - da0/6*(t-rS)^3;
        end
        % Find distance between predicted offset and each lane
        dLaneEgo = laneBounds-(frenetState(4)+Ldiff);
        
        % Determine future lane
        idx = find(dLaneEgo(2:(end-1)) >= 0 & dLaneEgo(3:(end)) < 0,1);
        if ~isempty(idx)
            laneNum(i) = idx;
        else
            laneNum(i) = 4;
        end
    end
end
end