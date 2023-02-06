function refPathOut = getReferencePath()
persistent refPath
if isempty(refPath)
%     roadCenters = [-43.7 52.9 0;
%     13.7 40.1 0;
%     57.1 14.5 0;
%     42.3 -63 0;
%     115.9 -120.8 0;
%     151 -131.2 0];
roadCenters = [-14.4 53.1 0;
    4.52 47.02 0;
    23.4 37.4 0;
    23.5 7.8 0;
    25.1 -27.6 0;
    42 -40.2 0;
    61.4 -39.2 0;
    71.8 -24 0;
    98.7 -20.5 0];
waypoints = roadCenters(:, 1:2);
refPath = referencePathFrenet(waypoints);
end
refPathOut = refPath;
end

