clc
clear
close all

delete('TP1_results_104580.txt')

% variables
NumMec = 104580;
Lcar = 0;           % Distância linear total percorrida na simulação pelo ego-veículo
Peds = 0;           % Número total de peões detetados
InPeds = 0;         % Número de peões detetados dentro da faixa onde circula o ego veículo
StopCars = 0;       % Número de veículos parados
MovCars = 0;        % Número de veículos em movimento
Bikes = 0;          % Número total de ciclistas
Lped1 = 0;          % Distância linear (desde o início da simulação) do primeiro peão encontrado na faixa
LStopCar1 = 0;      % Distância linear (desde o início da simulação) do primeiro veículo parado encontrado na estrada
LBarrFirst = 0;     % Distância linear (desde o início da simulação) do início da primeira barreira encontrada junto à estrada
LBarrLast = 0;      % Distância linear (desde o início da simulação) do fim da última barreira encontrada junto à estrada

% load the data
load('allData_104580.mat', 'allData');

PLOT_FLAG = 0;

% must be 1 to detect distances
flag_split_ObjDetect = 1; % use this flag to iterate over each detection (created to detect moving objects)

% sensor flags
CAMERA_ON = 1;
LEFT_RADAR_ON = 1;
RIGHT_RADAR_ON = 1;

% sensor index
camera_index = 1;
left_radar = 2;
right_radar = 3;
left_camera = 5;
right_camera = 6;

% total positions
tot_pos_camera = [];
tot_pos_left_radar = [];
tot_pos_right_radar = [];

% detections
detections_camera = [];
detections_left_radar = [];
detections_right_radar = [];

% counts
obj_count_camera = 0;
obj_count_left_radar = 0;
obj_count_right_radar = 0;

% num of poses by sensor
n_pos_camera = 1;
n_pos_left_radar = 1;
n_pos_right_radar = 1;

% poses of the Peds
peds_pos = [];
InPeds_flag = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% start analyse data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% count total num of objects PART 1 %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% use this to plot all trip
ti = 1;
tf = numel(allData);

PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', allData, 'UniformOutput', false))';
if PLOT_FLAG == 1
    plot(PP(:,1), PP(:,2), '.b')
    view(-90,90)
    axis equal
    hold on
end

for n=ti:tf
    egoCar_speed(n) = sqrt(allData(n).ActorPoses(1).Velocity(1)^2 + allData(n).ActorPoses(1).Velocity(2)^2);
    posCar=PP(n,:); % car position - vetor
    orCar=[
        allData(n).ActorPoses(1).Yaw allData(n).ActorPoses(1).Pitch allData(n).ActorPoses(1).Roll
        ]*pi/180; % car orientations - buscar os angulos e converter para radianos - em vetor
    TCtrans = trvec2tform(posCar); % posições em transformadas
    TCrot = eul2tform(orCar); % angulos para transformações geometricas
    Tcarro= TCtrans * TCrot;
    objs = allData(n).ObjectDetections;

    % transformadas do objecto
    for i=1:numel(objs)
        % parte importante
        posSens = objs{i}.Measurement'; % 6 vetor coord
        trvec = posSens(1:3); %  trans 11,22
        eul = posSens(4:6)*pi/180;
        % ----------------------
        trn = trvec2tform(trvec);
        rtn = eul2tform(eul);
        Tobj = trn*rtn;
        pos = Tcarro*Tobj*[0 0 0 1]';

        % analyse by sensor
        if objs{i}.SensorIndex == camera_index
            tot_pos_camera(n_pos_camera,:) = [pos(1) pos(2)];
            n_pos_camera = n_pos_camera + 1;

        elseif objs{i}.SensorIndex == left_radar
            tot_pos_left_radar(n_pos_left_radar,:) = [pos(1) pos(2)];
            n_pos_left_radar = n_pos_left_radar + 1;

        elseif objs{i}.SensorIndex == right_radar
            tot_pos_right_radar(n_pos_right_radar,:) = [pos(1) pos(2)];
            n_pos_right_radar = n_pos_right_radar + 1;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% remove the false detections
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% CAMERA
tot_pos_camera = sortrows(tot_pos_camera);
thresh = 2;
i=0;
for n=2:size(tot_pos_camera,1)
    if abs(tot_pos_camera(n,1)-tot_pos_camera(n-1,1)) < thresh && abs(tot_pos_camera(n,2)-tot_pos_camera(n-1,2)) < thresh
        if CAMERA_ON == 1 && PLOT_FLAG == 1
            plot(tot_pos_camera(n,1), tot_pos_camera(n,2), 'o')
        end
        i=i+1;
        detections_camera(i,:) = tot_pos_camera(n,:);
    end
end

%%%%%%%%%%%%%%%%%%%%% LEFT RADAR
tot_pos_left_radar = sortrows(tot_pos_left_radar);
thresh = 2;
i=0;
for n=2:size(tot_pos_left_radar,1)
    if abs(tot_pos_left_radar(n,1)-tot_pos_left_radar(n-1,1)) < thresh && abs(tot_pos_left_radar(n,2)-tot_pos_left_radar(n-1,2)) < thresh
        if LEFT_RADAR_ON == 1 && PLOT_FLAG == 1
            plot(tot_pos_left_radar(n,1), tot_pos_left_radar(n,2), 'o')
        end
        i=i+1;
        detections_left_radar(i,:) = tot_pos_left_radar(n,:);
    end
end

%%%%%%%%%%%%%%%%%%%%% RIGHT RADAR
tot_pos_right_radar = sortrows(tot_pos_right_radar);
thresh = 2;
i=0;
for n=2:size(tot_pos_right_radar,1)
    if abs(tot_pos_right_radar(n,1)-tot_pos_right_radar(n-1,1)) < thresh && abs(tot_pos_right_radar(n,2)-tot_pos_right_radar(n-1,2)) < thresh
        if RIGHT_RADAR_ON == 1 && PLOT_FLAG == 1
            plot(tot_pos_right_radar(n,1), tot_pos_right_radar(n,2), 'o')
        end
        i=i+1;
        detections_right_radar(i,:) = tot_pos_right_radar(n,:);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% find the total of objects detected
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% CAMERA
diff_obj_thresh = 2;
detections_camera = sortrows(detections_camera);
if ~isempty(detections_camera)
    obj_count_camera = 2;
    obj_camera(1) = 1;
    for n=2:size(detections_camera,1)
        if abs(detections_camera(n,1)-detections_camera(n-1,1))>diff_obj_thresh || abs(detections_camera(n,2)-detections_camera(n-1,2))>diff_obj_thresh
            obj_camera(obj_count_camera) = n;
            obj_camera(obj_count_camera+1) = size(detections_camera,1);
            if n~=size(detections_camera,1)
                obj_count_camera=obj_count_camera+1;
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%% LEFT RADAR
diff_obj_thresh = 5;
detections_left_radar = sortrows(detections_left_radar);
if ~isempty(detections_left_radar)
    obj_count_left_radar = 2;
    obj_left_radar(1) = 1;
    for n=2:size(detections_left_radar,1)
        if abs(detections_left_radar(n,1)-detections_left_radar(n-1,1))>diff_obj_thresh || abs(detections_left_radar(n,2)-detections_left_radar(n-1,2))>diff_obj_thresh
            obj_left_radar(obj_count_left_radar) = n;
            obj_left_radar(obj_count_left_radar+1) = size(detections_left_radar,1);
            if n~=size(detections_left_radar,1)
                obj_count_left_radar=obj_count_left_radar+1;
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%% RIGHT RADAR
diff_obj_thresh = 5;
detections_right_radar = sortrows(detections_right_radar);
if ~isempty(detections_right_radar)
    obj_count_right_radar = 2;
    obj_right_radar(1) = 1;
    for n=2:size(detections_right_radar,1)
        if abs(detections_right_radar(n,1)-detections_right_radar(n-1,1))>diff_obj_thresh || abs(detections_right_radar(n,2)-detections_right_radar(n-1,2))>diff_obj_thresh
            obj_right_radar(obj_count_right_radar) = n;
            obj_right_radar(obj_count_right_radar+1) = size(detections_right_radar,1);
            if n~=size(detections_right_radar,1)
                obj_count_right_radar=obj_count_right_radar+1;
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% divide the multiple objects into cell arrays and add rectangle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% CAMERA
if obj_count_camera > 2
    % most than 2 objects detected
    for n=2:obj_count_camera
        % divide the different objects in a cell array
        det_obj_camera{n-1}=detections_camera(obj_camera(n-1):obj_camera(n)-1,:);
        det_obj_camera{n}=detections_camera(obj_camera(n)-1:end,:);
        a = max(det_obj_camera{n-1});
        b = min(det_obj_camera{n-1});
        c=a-b;
        if size(c,2) == 2
            p=c(1)*c(2); % area of the object
            if CAMERA_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
                rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[1 0 0])
            end

%             if p<1.0 && p>0.5
%                 InPeds = InPeds + 1;
%             end
        end
    end
elseif obj_count_camera ~= 0
    % dectect only one object
    det_obj_camera{1}=detections_camera(obj_camera(1):end,:);
    a = max(det_obj_camera{1});
    b = min(det_obj_camera{1});
    c=a-b;
    if size(c,2) == 2
        p=c(1)*c(2); % area of the object
        if CAMERA_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
            rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[1 0 0])
        end

%         if p<1.0 && p>0.5
%             InPeds = InPeds + 1;
%         end
    end
end

%%%%%%%%%%%%%%%%%%%%% LEFT RADAR
if obj_count_left_radar > 2
    % most than 2 objects detected
    for n=2:obj_count_left_radar
        % divide the different objects in a cell array
        det_obj_left_radar{n-1}=detections_left_radar(obj_left_radar(n-1):obj_left_radar(n)-1,:);
        det_obj_left_radar{n}=detections_left_radar(obj_left_radar(n)-1:end,:);
        a = max(det_obj_left_radar{n-1});
        b = min(det_obj_left_radar{n-1});
        c=a-b;
        if size(c,2) == 2
            p=c(1)*c(2); % area of the object
            if LEFT_RADAR_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
                rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[0 1 0])
            end
            % categorize
            if p>1.0 && p<2.5
                Bikes = Bikes + 1;
            elseif p>8 && p<22
                StopCars = StopCars + 1;
            elseif p>0.17 && p<0.5
                Peds = Peds + 1;
                peds_pos(Peds,:) = [(a(1)+b(1))/2 (a(2)+b(2))/2];
            end

        end
    end
elseif obj_count_left_radar ~= 0
    % dectect only one object
    det_obj_left_radar{1}=detections_left_radar(obj_left_radar(1):end,:);
    a = max(det_obj_left_radar{1});
    b = min(det_obj_left_radar{1});
    c=a-b;
    if size(c,2) == 2
        p=c(1)*c(2); % area of the object
        if LEFT_RADAR_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
            rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[0 1 0])
        end
        % categorize
        if p>1.0 && p<2.5
            Bikes = Bikes + 1;
        elseif p>8 && p<22
            StopCars = StopCars + 1;
        elseif p>0.1 && p<0.5
            Peds = Peds + 1;
            peds_pos(Peds,:) = [(a(1)+b(1))/2 (a(2)+b(2))/2];
        end



    end
end

%%%%%%%%%%%%%%%%%%%%% RIGHT RADAR
if obj_count_right_radar > 2
    % most than 2 objects detected
    for n=2:obj_count_right_radar
        % divide the different objects in a cell array
        det_obj_right_radar{n-1}=detections_right_radar(obj_right_radar(n-1):obj_right_radar(n)-1,:);
        det_obj_right_radar{n}=detections_right_radar(obj_right_radar(n)-1:end,:);
        a = max(det_obj_right_radar{n-1});
        b = min(det_obj_right_radar{n-1});
        c=a-b;
        if size(c,2) == 2
            p=c(1)*c(2); % area of the object
            if RIGHT_RADAR_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
                rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[0 0 1])
            end
            % categorize
            if p>1.0 && p<2.5
                Bikes = Bikes + 1;
            elseif p>8 && p<22
                StopCars = StopCars + 1;
            elseif p>0.1 && p<0.5
                Peds = Peds + 1;
                peds_pos(Peds,:) = [(a(1)+b(1))/2 (a(2)+b(2))/2];
            end



        end
    end
elseif obj_count_right_radar ~= 0
    % dectect only one object
    det_obj_right_radar{1}=detections_right_radar(obj_right_radar(1):end,:);
    a = max(det_obj_right_radar{1});
    b = min(det_obj_right_radar{1});
    c=a-b;
    if size(c,2) == 2
        p=c(1)*c(2); % area of the object
        if RIGHT_RADAR_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
            rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[0 0 1])
        end
        %  categorize
        if p>1.0 && p<2.5
            Bikes = Bikes + 1;
        elseif p>8 && p<22
            StopCars = StopCars + 1;
        elseif p>0.1 && p<0.5
            Peds = Peds + 1;
            peds_pos(Peds,:) = [(a(1)+b(1))/2 (a(2)+b(2))/2];
        end
    end
end

% calculate linear distance traveled by the egocar
mean_egoCar_speed = mean(egoCar_speed);
Lcar = round(mean_egoCar_speed * allData(numel(allData)).Time,1);

% find the number of InPeds
for k=1:height(peds_pos)
    for n=1:numel(allData)
        egoCar_InPed_dist = sqrt((peds_pos(k,1)-PP(n,1))^2 + (peds_pos(k,2)-PP(n,2))^2);
        if egoCar_InPed_dist < 2.5 && InPeds_flag == 0
            InPeds = InPeds + 1;
            InPeds_flag = 1;
        end
        if egoCar_InPed_dist > 2.5
            InPeds_flag = 0;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% find distances to objects PART 2 %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars -except allData PLOT_FLAG flag_split_ObjDetect NumMec Lcar Peds InPeds StopCars MovCars Bikes Lped1 LStopCar1 LBarrFirst LBarrLast

% sensor flags
CAMERA_ON = 1;
LEFT_RADAR_ON = 1;
RIGHT_RADAR_ON = 1;
CAMERA_LEFT_ON = 1;
CAMERA_RIGHT_ON = 1;

% sensor index
camera_index = 1;
left_radar = 2;
right_radar = 3;
left_camera = 5;
right_camera = 6;

% total positions
tot_pos_camera = [];
tot_pos_left_radar = [];
tot_pos_right_radar = [];
tot_pos_left_camera = [];
tot_pos_right_camera = [];

% detections
detections_camera = [];
detections_left_radar = [];
detections_right_radar = [];
detections_left_camera = [];
detections_right_camera = [];

% counts
obj_count_camera = 0;
obj_count_left_radar = 0;
obj_count_right_radar = 0;
obj_count_left_camera = 0;
obj_count_right_camera = 0;

% num of poses by sensor
n_pos_camera = 1;
n_pos_left_radar = 1;
n_pos_right_radar = 1;
n_pos_left_camera = 1;
n_pos_right_camera = 1;

% flag first barrier
first_ber = 0;
obj_dim_max = 0;
obj_dim_max_last = 0;
first_car = 0;
first_ped = 0;

% find index where ObjectDetections has data
if flag_split_ObjDetect == 1
    i = 0;
    for n = 1:numel(allData)
        if ~isempty(allData(n).ObjectDetections)
            i = i+1;
            time_det(i) = n;
        end
    end

    % analyse the data of the sensors
    for time = 1:size(time_det,2)

        % define time to plot
        if PLOT_FLAG == 1
            sprintf('Det: %.0f of %.0f', time, size(time_det,2))
            %         pause
            clf;
        end
        %         ti = time_det(time);
        if time > 2
            ti = time_det(time-2);
        else
            ti = time_det(time);
        end

        tf = time_det(time);

        % num of poses by sensor
        n_pos_camera = 1;
        n_pos_left_radar = 1;
        n_pos_right_radar = 1;
        n_pos_left_camera = 1;
        n_pos_right_camera = 1;

        PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', allData, 'UniformOutput', false))';
        if PLOT_FLAG == 1
            plot(PP(:,1), PP(:,2), '.b')
            view(-90,90)
            axis equal
            hold on
        end


        for n=ti:tf
            if PLOT_FLAG == 1
                plot(PP(n,1),PP(n,2),'*k')
            end
            egoCar_speed(n) = sqrt(allData(n).ActorPoses(1).Velocity(1)^2 + allData(n).ActorPoses(1).Velocity(2)^2);
            posCar=PP(n,:); % car position - vetor
            orCar=[
                allData(n).ActorPoses(1).Yaw allData(n).ActorPoses(1).Pitch allData(n).ActorPoses(1).Roll
                ]*pi/180; % car orientations - buscar os angulos e converter para radianos - em vetor
            TCtrans = trvec2tform(posCar); % posições em transformadas
            TCrot = eul2tform(orCar); % angulos para transformações geometricas
            Tcarro= TCtrans * TCrot;
            objs = allData(n).ObjectDetections;

            % transformadas do objecto
            for i=1:numel(objs)
                % parte importante
                posSens = objs{i}.Measurement'; % 6 vetor coord
                trvec = posSens(1:3); %  trans 11,22
                eul = posSens(4:6)*pi/180;
                % ----------------------
                trn = trvec2tform(trvec);
                rtn = eul2tform(eul);
                Tobj = trn*rtn;
                pos = Tcarro*Tobj*[0 0 0 1]';

                % analyse by sensor
                if objs{i}.SensorIndex == camera_index
                    tot_pos_camera(n_pos_camera,:) = [pos(1) pos(2)];
                    n_pos_camera = n_pos_camera + 1;

                elseif objs{i}.SensorIndex == left_radar
                    tot_pos_left_radar(n_pos_left_radar,:) = [pos(1) pos(2)];
                    n_pos_left_radar = n_pos_left_radar + 1;

                elseif objs{i}.SensorIndex == right_radar
                    tot_pos_right_radar(n_pos_right_radar,:) = [pos(1) pos(2)];
                    n_pos_right_radar = n_pos_right_radar + 1;

                elseif objs{i}.SensorIndex == left_camera
                    tot_pos_left_camera(n_pos_left_camera,:) = [pos(1) pos(2)];
                    n_pos_left_camera = n_pos_left_camera + 1;

                elseif objs{i}.SensorIndex == right_camera
                    tot_pos_right_camera(n_pos_right_camera,:) = [pos(1) pos(2)];
                    n_pos_right_camera = n_pos_right_camera + 1;
                end
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % remove the false detections
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%% CAMERA
        tot_pos_camera = sortrows(tot_pos_camera);
        thresh = 2;
        i=0;
        for n=2:size(tot_pos_camera,1)
            if abs(tot_pos_camera(n,1)-tot_pos_camera(n-1,1)) < thresh && abs(tot_pos_camera(n,2)-tot_pos_camera(n-1,2)) < thresh
                if CAMERA_ON == 1 && PLOT_FLAG == 1
                    plot(tot_pos_camera(n,1), tot_pos_camera(n,2), 'o')
                end
                i=i+1;
                detections_camera(i,:) = tot_pos_camera(n,:);
            end
        end

        %%%%%%%%%%%%%%%%%%%%% LEFT RADAR
        tot_pos_left_radar = sortrows(tot_pos_left_radar);
        thresh = 2;
        i=0;
        for n=2:size(tot_pos_left_radar,1)
            if abs(tot_pos_left_radar(n,1)-tot_pos_left_radar(n-1,1)) < thresh && abs(tot_pos_left_radar(n,2)-tot_pos_left_radar(n-1,2)) < thresh
                if LEFT_RADAR_ON == 1 && PLOT_FLAG == 1
                    plot(tot_pos_left_radar(n,1), tot_pos_left_radar(n,2), 'o')
                end
                i=i+1;
                detections_left_radar(i,:) = tot_pos_left_radar(n,:);
            end
        end

        %%%%%%%%%%%%%%%%%%%%% RIGHT RADAR
        tot_pos_right_radar = sortrows(tot_pos_right_radar);
        thresh = 2;
        i=0;
        for n=2:size(tot_pos_right_radar,1)
            if abs(tot_pos_right_radar(n,1)-tot_pos_right_radar(n-1,1)) < thresh && abs(tot_pos_right_radar(n,2)-tot_pos_right_radar(n-1,2)) < thresh
                if RIGHT_RADAR_ON == 1 && PLOT_FLAG == 1
                    plot(tot_pos_right_radar(n,1), tot_pos_right_radar(n,2), 'o')
                end
                i=i+1;
                detections_right_radar(i,:) = tot_pos_right_radar(n,:);
            end
        end

        %%%%%%%%%%%%%%%%%%%%% LEFT CAMERA
        tot_pos_left_camera = sortrows(tot_pos_left_camera);
        thresh = 2;
        i=0;
        if tot_pos_left_camera > 1
            for n=2:size(tot_pos_left_camera,1)
                if abs(tot_pos_left_camera(n,1)-tot_pos_left_camera(n-1,1)) < thresh && abs(tot_pos_left_camera(n,2)-tot_pos_left_camera(n-1,2)) < thresh
                    if CAMERA_LEFT_ON == 1 && PLOT_FLAG == 1
                        plot(tot_pos_left_camera(n,1), tot_pos_left_camera(n,2), 'o')
                    end
                    i=i+1;
                    detections_left_camera(i,:) = tot_pos_left_camera(n,:);
                end
            end
        else
            detections_left_camera = tot_pos_left_camera;
        end

        %%%%%%%%%%%%%%%%%%%%% RIGHT CAMERA
        tot_pos_right_camera = sortrows(tot_pos_right_camera);
        thresh = 2;
        i=0;
        if tot_pos_right_camera > 1
            for n=2:size(tot_pos_right_camera,1)
                if abs(tot_pos_right_camera(n,1)-tot_pos_right_camera(n-1,1)) < thresh && abs(tot_pos_right_camera(n,2)-tot_pos_right_camera(n-1,2)) < thresh
                    if CAMERA_RIGHT_ON == 1 && PLOT_FLAG == 1
                        plot(tot_pos_right_camera(n,1), tot_pos_right_camera(n,2), 'o')
                    end
                    i=i+1;
                    detections_right_camera(i,:) = tot_pos_right_camera(n,:);
                end
            end
        else
            detections_right_camera = tot_pos_right_camera;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % find the total of objects detected
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%% CAMERA
        diff_obj_thresh = 5;
        detections_camera = sortrows(detections_camera);
        if ~isempty(detections_camera)
            obj_count_camera = 2;
            obj_camera(1) = 1;
            for n=2:size(detections_camera,1)
                if abs(detections_camera(n,1)-detections_camera(n-1,1))>diff_obj_thresh || abs(detections_camera(n,2)-detections_camera(n-1,2))>diff_obj_thresh
                    obj_camera(obj_count_camera) = n;
                    obj_camera(obj_count_camera+1) = size(detections_camera,1);
                    if n~=size(detections_camera,1)
                        obj_count_camera=obj_count_camera+1;
                    end
                end
            end
        end

        %%%%%%%%%%%%%%%%%%%%% LEFT RADAR
        diff_obj_thresh = 5;
        detections_left_radar = sortrows(detections_left_radar);
        if ~isempty(detections_left_radar)
            obj_count_left_radar = 2;
            obj_left_radar(1) = 1;
            for n=2:size(detections_left_radar,1)
                if abs(detections_left_radar(n,1)-detections_left_radar(n-1,1))>diff_obj_thresh || abs(detections_left_radar(n,2)-detections_left_radar(n-1,2))>diff_obj_thresh
                    obj_left_radar(obj_count_left_radar) = n;
                    obj_left_radar(obj_count_left_radar+1) = size(detections_left_radar,1);
                    if n~=size(detections_left_radar,1)
                        obj_count_left_radar=obj_count_left_radar+1;
                    end
                end
            end
        end

        %%%%%%%%%%%%%%%%%%%%% RIGHT RADAR
        diff_obj_thresh = 5;
        detections_right_radar = sortrows(detections_right_radar);
        if ~isempty(detections_right_radar)
            obj_count_right_radar = 2;
            obj_right_radar(1) = 1;
            for n=2:size(detections_right_radar,1)
                if abs(detections_right_radar(n,1)-detections_right_radar(n-1,1))>diff_obj_thresh || abs(detections_right_radar(n,2)-detections_right_radar(n-1,2))>diff_obj_thresh
                    obj_right_radar(obj_count_right_radar) = n;
                    obj_right_radar(obj_count_right_radar+1) = size(detections_right_radar,1);
                    if n~=size(detections_right_radar,1)
                        obj_count_right_radar=obj_count_right_radar+1;
                    end
                end
            end
        end

        %%%%%%%%%%%%%%%%%%%%% LEFT CAMERA
        diff_obj_thresh = 5;
        detections_left_camera = sortrows(detections_left_camera);
        if ~isempty(detections_left_camera)
            obj_count_left_camera = 2;
            obj_left_camera(1) = 1;
            for n=2:size(detections_left_camera,1)
                if abs(detections_left_camera(n,1)-detections_left_camera(n-1,1))>diff_obj_thresh || abs(detections_left_camera(n,2)-detections_left_camera(n-1,2))>diff_obj_thresh
                    obj_left_camera(obj_count_left_camera) = n;
                    obj_left_camera(obj_count_left_camera+1) = size(detections_left_camera,1);
                    if n~=size(detections_left_camera,1)
                        obj_count_left_camera=obj_count_left_camera+1;
                    end
                end
            end
        end

        %%%%%%%%%%%%%%%%%%%%% RIGHT CAMERA
        diff_obj_thresh = 5;
        detections_right_camera = sortrows(detections_right_camera);
        if ~isempty(detections_right_camera)
            obj_count_right_camera = 2;
            obj_right_camera(1) = 1;
            for n=2:size(detections_right_camera,1)
                if abs(detections_right_camera(n,1)-detections_right_camera(n-1,1))>diff_obj_thresh || abs(detections_right_camera(n,2)-detections_right_camera(n-1,2))>diff_obj_thresh
                    obj_right_camera(obj_count_right_camera) = n;
                    obj_right_camera(obj_count_right_camera+1) = size(detections_right_camera,1);
                    if n~=size(detections_right_camera,1)
                        obj_count_right_camera=obj_count_right_camera+1;
                    end
                end
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % divide the multiple objects into cell arrays and add rectangle
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%% CAMERA
        if obj_count_camera > 2
            % most than 2 objects detected
            for n=2:obj_count_camera
                % divide the different objects in a cell array
                det_obj_camera{n-1}=detections_camera(obj_camera(n-1):obj_camera(n)-1,:);
                det_obj_camera{n}=detections_camera(obj_camera(n)-1:end,:);
                a = max(det_obj_camera{n-1});
                b = min(det_obj_camera{n-1});
                c=a-b;
                if size(c,2) == 2
                    p=c(1)*c(2); % area of the object
                    if CAMERA_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
                        rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[1 0 0])
                    end
                end

            end
        elseif obj_count_camera ~= 0
            % dectect only one object
            det_obj_camera{1}=detections_camera(obj_camera(1):end,:);
            a = max(det_obj_camera{1});
            b = min(det_obj_camera{1});
            c=a-b;
            if size(c,2) == 2
                p=c(1)*c(2); % area of the object
                if CAMERA_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
                    rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[1 0 0])
                end
            end

        end

        %%%%%%%%%%%%%%%%%%%%% LEFT RADAR
        if obj_count_left_radar > 2
            % most than 2 objects detected
            for n=2:obj_count_left_radar
                % divide the different objects in a cell array
                det_obj_left_radar{n-1}=detections_left_radar(obj_left_radar(n-1):obj_left_radar(n)-1,:);
                det_obj_left_radar{n}=detections_left_radar(obj_left_radar(n)-1:end,:);
                a = max(det_obj_left_radar{n-1});
                b = min(det_obj_left_radar{n-1});
                c=a-b;
                if size(c,2) == 2 && ~isempty(c)
                    obj_dim = sqrt((a(2)-b(2))^2+(a(1)-b(1))^2);
                    p=c(1)*c(2); % area of the object
                    if LEFT_RADAR_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
                        rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[0 1 0])
                    end

                    % detect first pedestrian
                    if p>0.3 && p<0.5
                        if first_ped == 0 && ~isempty(detections_left_camera)
                            first_ped = 1;
                            mean_egoCar_speed_1st_ped = mean(egoCar_speed);
                            Lped1 = mean_egoCar_speed_1st_ped * allData(tf).Time;
                        end
                    end


                    % detect first car
                    if (c(1) > 3 && c(1) < 6) || (c(2) > 3 && c(2) < 6)
                        if (c(1)/c(2) > 1.4 && c(1)/c(2) < 3) || (c(2)/c(1) > 1.3 && c(2)/c(1) < 3)
                            if obj_dim > 3.5 && obj_dim < 5.8
                                if first_car == 0 && ~isempty(detections_left_camera)
                                    mean_egoCar_speed_1st_car = mean(egoCar_speed);
                                    LStopCar1 = mean_egoCar_speed_1st_car * allData(tf).Time;
                                    first_car = 1;
                                end
                            end
                        end
                    end

                    % detect first berrier
                    if obj_dim > 9
                        if obj_dim_max < obj_dim
                            obj_dim_max = obj_dim;
                            if first_ber == 0
                                mean_egoCar_speed_1st_ber = mean(egoCar_speed);
                                LBarrFirst = mean_egoCar_speed_1st_ber * allData(tf).Time;
                            end
                        end
                        if obj_dim < obj_dim_max
                            first_ber = 1;
                        end
                    end

                    % detect last barrier
                    if obj_dim > 9
                        if obj_dim_max_last < obj_dim
                            obj_dim_max_last = obj_dim;
                            mean_egoCar_speed_last_ber = mean(egoCar_speed);
                            LBarrLast = mean_egoCar_speed_last_ber * allData(tf).Time;
                        end
                    end
                end
            end
        elseif obj_count_left_radar ~= 0
            % dectect only one object
            det_obj_left_radar{1}=detections_left_radar(obj_left_radar(1):end-1,:);
            a = max(det_obj_left_radar{1});
            b = min(det_obj_left_radar{1});
            c=a-b;
            if size(c,2) == 2 && ~isempty(c)
                obj_dim = sqrt((a(2)-b(2))^2+(a(1)-b(1))^2);
                p=c(1)*c(2); % area of the object
                if LEFT_RADAR_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
                    rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[0 1 0])
                end

                % detect first car
                if (c(1) > 3 && c(1) < 6) || (c(2) > 3 && c(2) < 6)
                    if (c(1)/c(2) > 1.4 && c(1)/c(2) < 3) || (c(2)/c(1) > 1.3 && c(2)/c(1) < 3)
                        if obj_dim > 3.5 && obj_dim < 5.8
                            if first_car == 0 && ~isempty(detections_left_camera)
                                mean_egoCar_speed_1st_car = mean(egoCar_speed);
                                LStopCar1 = mean_egoCar_speed_1st_car * allData(tf).Time;
                                first_car = 1;
                            end
                        end
                    end
                end

                % detect first berrier
                if obj_dim > 9
                    if obj_dim_max < obj_dim
                        obj_dim_max = obj_dim;
                        if first_ber == 0
                            mean_egoCar_speed_1st_ber = mean(egoCar_speed);
                            LBarrFirst = mean_egoCar_speed_1st_ber * allData(tf).Time;
                        end
                    end
                    if obj_dim < obj_dim_max
                        first_ber = 1;
                    end
                end

                % detect last barrier
                if obj_dim > 9
                    if obj_dim_max_last < obj_dim
                        obj_dim_max_last = obj_dim;
                        mean_egoCar_speed_last_ber = mean(egoCar_speed);
                        LBarrLast = mean_egoCar_speed_last_ber * allData(tf).Time;
                    end
                end
            end
        end

        %%%%%%%%%%%%%%%%%%%%% RIGHT RADAR
        if obj_count_right_radar > 2
            % most than 2 objects detected
            for n=2:obj_count_right_radar
                % divide the different objects in a cell array
                det_obj_right_radar{n-1}=detections_right_radar(obj_right_radar(n-1):obj_right_radar(n)-1,:);
                det_obj_right_radar{n}=detections_right_radar(obj_right_radar(n)-1:end,:);
                a = max(det_obj_right_radar{n-1});
                b = min(det_obj_right_radar{n-1});
                c=a-b;
                if size(c,2) == 2 && ~isempty(c)
                    obj_dim = sqrt((a(2)-b(2))^2+(a(1)-b(1))^2);
                    p=c(1)*c(2); % area of the object
                    if RIGHT_RADAR_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
                        rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[0 0 1])
                    end

                    % detect first car
                    if (c(1) > 3 && c(1) < 6) || (c(2) > 3 && c(2) < 6)
                        if (c(1)/c(2) > 1.4 && c(1)/c(2) < 3) || (c(2)/c(1) > 1.3 && c(2)/c(1) < 3)
                            if obj_dim > 3.5 && obj_dim < 5.8
                                if first_car == 0 && ~isempty(detections_right_camera)
                                    mean_egoCar_speed_1st_car = mean(egoCar_speed);
                                    LStopCar1 = mean_egoCar_speed_1st_car * allData(tf).Time;
                                    first_car = 1;
                                end
                            end
                        end
                    end

                    % detect first berrier
                    if obj_dim > 9
                        if obj_dim_max < obj_dim
                            obj_dim_max = obj_dim;
                            if first_ber == 0
                                mean_egoCar_speed_1st_ber = mean(egoCar_speed);
                                LBarrFirst = mean_egoCar_speed_1st_ber * allData(tf).Time;
                            end
                        end
                        if obj_dim < obj_dim_max
                            first_ber = 1;
                        end
                    end

                    % detect last barrier
                    if obj_dim > 9
                        if obj_dim_max_last < obj_dim
                            obj_dim_max_last = obj_dim;
                            mean_egoCar_speed_last_ber = mean(egoCar_speed);
                            LBarrLast = mean_egoCar_speed_last_ber * allData(tf).Time;
                        end
                    end
                end
            end
        elseif obj_count_right_radar ~= 0
            % dectect only one object
            det_obj_right_radar{1}=detections_right_radar(obj_right_radar(1):end-1,:);
            a = max(det_obj_right_radar{1});
            b = min(det_obj_right_radar{1});
            c=a-b;
            if size(c,2) == 2 && ~isempty(c)
                obj_dim = sqrt((a(2)-b(2))^2+(a(1)-b(1))^2);
                p=c(1)*c(2); % area of the object
                if RIGHT_RADAR_ON == 1 && max(c) < 10 && PLOT_FLAG == 1
                    rectangle('Position',[b(1) b(2) c(1) c(2)],'FaceColor',[0 0 1])
                end

                % detect first car
                if (c(1) > 3 && c(1) < 6) || (c(2) > 3 && c(2) < 6)
                    if (c(1)/c(2) > 1.4 && c(1)/c(2) < 3) || (c(2)/c(1) > 1.3 && c(2)/c(1) < 3)
                        if obj_dim > 3.5 && obj_dim < 5.8
                            if first_car == 0 && ~isempty(detections_right_camera)
                                mean_egoCar_speed_1st_car = mean(egoCar_speed);
                                LStopCar1 = mean_egoCar_speed_1st_car * allData(tf).Time;
                                first_car = 1;
                            end
                        end
                    end
                end

                % detect first berrier
                if obj_dim > 9
                    if obj_dim_max < obj_dim
                        obj_dim_max = obj_dim;
                        if first_ber == 0
                            mean_egoCar_speed_1st_ber = mean(egoCar_speed);
                            LBarrFirst = mean_egoCar_speed_1st_ber * allData(tf).Time;
                        end
                    end
                    if obj_dim < obj_dim_max
                        first_ber = 1;
                    end
                end

                % detect last barrier
                if obj_dim > 9
                    if obj_dim_max_last < obj_dim
                        obj_dim_max_last = obj_dim;
                        mean_egoCar_speed_last_ber = mean(egoCar_speed);
                        LBarrLast = mean_egoCar_speed_last_ber * allData(tf).Time;
                    end
                end
            end
        end
    end
end

LBarrFirst = round(LBarrFirst,1);
LBarrLast = round(LBarrLast,1);
LStopCar1 = round(LStopCar1,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% write output file TP1_results_104580.txt
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
file = fopen(['TP1_results_' num2str(NumMec) '.txt'], 'a');
text_line = strcat(num2str(NumMec),',',num2str(Lcar),',',num2str(Peds),',',num2str(InPeds),',',num2str(StopCars),',',num2str(MovCars),',',num2str(Bikes) ...
    ,',',num2str(Lped1),',',num2str(LStopCar1),',',num2str(LBarrFirst),',',num2str(LBarrLast));
fprintf(file, text_line);
fclose(file);