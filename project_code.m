mapLayers.StationaryObstacles = imread('stationary.jpg');
mapLayers.StationaryObstacles = im2gray(mapLayers.StationaryObstacles);
mapLayers.StationaryObstacles = imcomplement(mapLayers.StationaryObstacles);
mapLayers.RoadMarkings = imread("road_markings.jpg");
mapLayers.RoadMarkings = im2gray(mapLayers.RoadMarkings);
mapLayers.RoadMarkings = imcomplement(mapLayers.RoadMarkings);
mapLayers.ParkedCars = imread("parked_cars.jpg");
mapLayers.ParkedCars = im2gray(mapLayers.ParkedCars);
mapLayers.ParkedCars = imcomplement(mapLayers.ParkedCars);

% plotMapLayers(mapLayers);

combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + mapLayers.ParkedCars;
combinedMap = im2single(combinedMap);

res = 0.02083;
costmap = vehicleCostmap(combinedMap,'CellSize',res);

resolution = 1/costmap.CellSize;
omap = occupancyMap(combinedMap,resolution);
omap.FreeThreshold = costmap.FreeThreshold;
omap.OccupiedThreshold = costmap.OccupiedThreshold;
refFigure = figure('Name','Parking Lot Map');
show(omap)

startPose = [0.5 4 pi/6];
goalPose = [10.5 15 pi/2];
plotRobot(startPose,"red")

amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
amcl.UpdateThresholds = [0.1,0.1,0.1];
amcl.SensorModel.Map = omap;

initialEstimate = [0 4 0];

amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;
amcl.InitialPose = initialEstimate;
amcl.InitialCovariance = eye(3)*0.5;

visualizationHelper = ExampleHelperAMCLVisualization(omap);

numUpdates = 24;
i = 1;
while i <= numUpdates
    %create a lidar scan
    sensor = rangeSensor;
    sensor.Range = [0.45 8];
    sensor.HorizontalAngle = [-pi/12 pi/12];
    position = [startPose(1:2) (startPose(3)+(i*pi/6))];
    [ranges,angles] = sensor(position,omap);
    scan = lidarScan(ranges,angles);

    %update estimated robot's pose and covariance using new odometry and
    %sensor readings.
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(position,scan);

    % plot the robot's estimated pose, particles and laser scans on the
    % map.
    if isUpdated
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
    end
    i = i+1;
end

%Path Planning using Hybrid A Star Algorithm
validator = validatorOccupancyMap;
validator.Map = omap;
planner = plannerHybridAStar(validator,'MinTurningRadius',.667,'MotionPrimitiveLength',1);
refPath = plan(planner,estimatedPose,goalPose);
figure
show(planner)
plotRobot(startPose,"red")
plotRobot(goalPose,"green")

function plotMapLayers(mapLayers)
figure
cellOfMaps = cellfun(@imcomplement, struct2cell(mapLayers), 'UniformOutput', false);
montage(cellOfMaps,'Size',[1 numel(cellOfMaps)], 'Border', [5 5], 'ThumbnailSize', [300 NaN])
title('Map Layers - Stationary Obstacles, Road markings, and Parked Cars')
end

function plotRobot(startPose,color)
    angles = startPose(3);
    trans = [repmat([startPose(:,1:2) 0], length(angles), 1)];
    rot = axang2quat([repmat([0 0 1],length(angles),1) angles']);
    hold on
    plotTransforms(trans,rot,'MeshFilePath','groundvehicle.stl','MeshColor',color)
    view(0,90)
    hold off
end