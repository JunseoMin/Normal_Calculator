clc;
close all;
clear variables; %clear classes;

%% Load depth normal dataset
depthNormalData1 = readmatrix("./data/output/pm_normal.csv");
depthNormalData2 = readmatrix("./data/output/pw_normal.csv");

nx1 = depthNormalData1(:,1);
ny1 = depthNormalData1(:,2);
nz1 = depthNormalData1(:,3);

nx2 = depthNormalData2(:,1);
ny2 = depthNormalData2(:,2);
nz2 = depthNormalData2(:,3);

%% Input Params Setting
tracking_num = 48;

R = [0.5455, 0.145]; % real hatch radius
pm_position = [10.9349; -10.1; 5.2508];
pm_normal = [0; 1; 0];
T_true_p2c_iss = [eye(3), pm_position; 0 0 0 1];
visualize_colmap = 1; % optional. 

focalLength    = [608.210845 608.210845]; 
principalPoint = [640 440];
imageSize      = [1280 880];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

ellipses_params = cell(1, tracking_num);
PM_normal_cam = cell(1, tracking_num);
PM_center_cam = cell(1, tracking_num);
Window_normal_cam = cell(1, tracking_num);
Window_center_cam = cell(1, tracking_num);
C_pm = cell(1, tracking_num);
C_win = cell(1, tracking_num);
ransac = ones(1, tracking_num);
angle_diff = cell(1, tracking_num);

ellipse_result1 = [];
ellipse_result2 = [];

ellipsePath = "data/AAMED/";
fileList = dir(fullfile(ellipsePath, '*.txt'));

for i = 1:length(fileList)
    filename = fullfile(ellipsePath, fileList(i).name);
    two_ellipse_result = readmatrix(filename);

    ellipse1 = 0; ellipse2 = 0;

    for j = 1:size(two_ellipse_result,1)
        if two_ellipse_result(j,2) ~= 0
            if two_ellipse_result(j,7) == 0
                ellipse1 = two_ellipse_result(j,1:6);
            elseif two_ellipse_result(j,7) == 1
                ellipse2 = two_ellipse_result(j,1:6);
            end
        end
    end

    if ellipse1(1) ~= 0 && ellipse2(1) ~= 0
        ellipse_result1 = [ellipse_result1; ellipse1];
        ellipse_result2 = [ellipse_result2; ellipse2];
    end
end

%% P2C
for i = 1:tracking_num
    ellipse_params1 = ellipse_result1(i, 2:end);
    ellipse_params2 = ellipse_result2(i, 2:end);
    ellipses_params{i} = [ellipse_params1; ellipse_params2];
    
    [PM_normal_cam{i}, PM_center_cam{i}, Window_normal_cam{i}, Window_center_cam{i}, C_pm{i}, C_win{i}, angle_diff{i}] = perspective_two_circle(ellipses_params{i}, R, intrinsics.K);
end

%% Visualize P2C normal VS Depth normal
nFrames = min([tracking_num, ...
               size(depthNormalData1,1), size(depthNormalData2,1), ...
               numel(PM_normal_cam), numel(Window_normal_cam), ...
               numel(PM_center_cam), numel(Window_center_cam)]);
if nFrames < tracking_num
    warning('Using %d frames (limited by available data)', nFrames);
end

normalize_vec = @(v) (v ./ max(eps, vecnorm(v,2,1)));   % column-wise normalize
ang_deg = @(a,b) rad2deg(atan2(vecnorm(cross(a,b,1),2,1), max(eps, dot(a,b,1)))); % 0~180

ang_pm = zeros(1, nFrames);
ang_win = zeros(1, nFrames);

figure('Color','w','Position',[100 100 1200 540]);

subplot(1,2,1); hold on; grid on; axis equal;
title('PM normals @ PM\_center\_cam','Interpreter','none');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45,25);

scale = 0.5;

for i = 1:nFrames
    nd_pm = [nx1(i); ny1(i); nz1(i)];
    np_pm = PM_normal_cam{i}(:);
    cp_pm = PM_center_cam{i}(:);

    nd_pm = normalize_vec(nd_pm);
    np_pm = normalize_vec(np_pm);

    if dot(nd_pm, np_pm) < 0, np_pm = -np_pm; end

    ang_pm(i) = ang_deg(nd_pm, np_pm);

    quiver3(cp_pm(1), cp_pm(2), cp_pm(3), nd_pm(1)*scale, nd_pm(2)*scale, nd_pm(3)*scale, ...
        'LineWidth',1.5,'MaxHeadSize',0.6,'Color',[0 0.45 0.85]); % depth: blue-ish
    quiver3(cp_pm(1), cp_pm(2), cp_pm(3), np_pm(1)*scale, np_pm(2)*scale, np_pm(3)*scale, ...
        'LineWidth',1.5,'MaxHeadSize',0.6,'Color',[0.2 0.7 0.2]); % p2c: green

    plot3(cp_pm(1), cp_pm(2), cp_pm(3), '.', 'Color',[0.5 0.5 0.5]);
end
legend({'Depth normal','P2C normal','Center'}, 'Location','best');

subplot(1,2,2); hold on; grid on; axis equal;
title('Window normals @ Window\_center\_cam','Interpreter','none');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45,25);

for i = 1:nFrames
    nd_w = [nx2(i); ny2(i); nz2(i)];
    np_w = Window_normal_cam{i}(:);
    cp_w = Window_center_cam{i}(:);

    nd_w = normalize_vec(nd_w);
    np_w = normalize_vec(np_w);
    if dot(nd_w, np_w) < 0, np_w = -np_w; end

    ang_win(i) = ang_deg(nd_w, np_w);

    quiver3(cp_w(1), cp_w(2), cp_w(3), nd_w(1)*scale, nd_w(2)*scale, nd_w(3)*scale, ...
        'LineWidth',1.5,'MaxHeadSize',0.6,'Color',[0 0.45 0.85]); % depth
    quiver3(cp_w(1), cp_w(2), cp_w(3), np_w(1)*scale, np_w(2)*scale, np_w(3)*scale, ...
        'LineWidth',1.5,'MaxHeadSize',0.6,'Color',[0.2 0.7 0.2]); % p2c

    plot3(cp_w(1), cp_w(2), cp_w(3), '.', 'Color',[0.5 0.5 0.5]);
end
legend({'Depth normal','P2C normal','Center'}, 'Location','best');

figure('Color','w','Position',[100 680 1200 320]); 
t = 1:nFrames;
plot(t, ang_pm, '-o','LineWidth',1.5,'MarkerSize',5); hold on;
plot(t, ang_win,'-s','LineWidth',1.5,'MarkerSize',5);
yline(mean(ang_pm),'--'); yline(mean(ang_win),'--');
grid on; xlabel('Frame index'); ylabel('Angle (deg)');
title('Angle between Depth normal and P2C normal (per frame)');
legend({'PM angle','Window angle','PM mean','Window mean'}, 'Location','best');

fprintf('[Summary] PM  : mean=%.3f deg, median=%.3f deg, max=%.3f deg\n', ...
        mean(ang_pm), median(ang_pm), max(ang_pm));
fprintf('[Summary] WIN : mean=%.3f deg, median=%.3f deg, max=%.3f deg\n', ...
        mean(ang_win), median(ang_win), max(ang_win));
