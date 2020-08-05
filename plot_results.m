%% Stereo Visual Odometry Result Visualization ~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{

This script is the main code for the Master Thesis Project "Sparse Visual
Odometry with Local Non-Linear Least-Squares Optimization for Navigation of
Autonomous Vehicles" [1] results visualization.

This code is a stereo vision pipeline which uses the KITTI dataset [2] as
input data. 
 
For further reading, please refer to [1].

NOTE: This script only works if the directory "/results" exist and if files
with results exists inside the results folder...

Author:         Fabian Aguilar
Date:           01/27/19
Edition:        3
Latest Edition: 08/03/20

Reference: [1] E. F. Aguilar Calzadillas, "Sparse Stereo Visual Odometry 
           with Local Non-Linear Least-Squares Optimization for 
           Navigation of Autonomous Vehicles", M.A.Sc. Thesis, 
           Depart. of Mech. and Aero. Eng., Carleton University, 
           Ottawa ON, Canada, 2019. [Online] Available: 
           https://curve.carleton.ca/7270ba62-1fd3-4f1b-a1fa-6031b06585e9

           [2] A. Geiger, “The KITTI Vision Benchmark Suit,” cvlibs.net, 
           2019. [Online]. Available: http://www.cvlibs.net/datasets/kitti/
%}
%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
close all; clear all; clc;
warning('off')

if exist('results', 'dir')
    addpath(genpath('results'));
else
    error('/reuslts directory does not exist...')
end
    

%% Settings
% Plot Bundle Adjusted results?
settings.BundleAdjustment = true;
% Marker Sizes
settings.CameraSize = 5;
settings.marker = 10;
settings.markerLine = 5;
settings.LineWidth = 1.5;
settings.TitleFontSize = 12;
settings.LabelFontSize = 10;
settings.LegendFontSize = 10;
settings.LegendLocation = 'northwest';

%% Load results
% Make sure to have the right files here
settings.sequence = '03';                      % Number of the sequence for plot titles 
results = load('KITTI_Seq_03.mat');            % Make sure to enter the right name

%% Retrive View Sets
GTset = results.dataset.vGTset;

% VO Left and Right Track with Local Optimization
VOset_LR_LO.VO = results.SVO.vVOset;
VOset_LR_LO.VO_BA = results.SVO.vVOsetBA;

%% Translational and Rotational error 
[VOset_LR_LO.TR_error_VO] = getTranslationRotationError(GTset, VOset_LR_LO.VO);

% Bundle Adjusted
if settings.BundleAdjustment
    [VOset_LR_LO.TR_error_VO_BA] = getTranslationRotationError(GTset, VOset_LR_LO.VO_BA);
end

%% Error in XYZ Translation
[VOset_LR_LO.T_error_VO, VOset_LR_LO.T_ABSerror_VO] = getTranslationError(GTset, VOset_LR_LO.VO);

% Bundle Adjusted
if settings.BundleAdjustment
    [VOset_LR_LO.T_error_VO_BA, VOset_LR_LO.T_ABSerror_VO_BA] = getTranslationError(GTset, VOset_LR_LO.VO_BA);
end

%% Odometry
[VOset_LR_LO.Odometry] = getEstimatedOdomery(GTset, VOset_LR_LO.VO);

if settings.BundleAdjustment
    [VOset_LR_LO.OdometryBA] = getEstimatedOdomery(GTset, VOset_LR_LO.VO_BA);
end

%% Plot Results
plotResults(GTset, VOset_LR_LO, settings)

fprintf('Data has been plotted... ')

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

%% Helper Functions
function plotResults(GTset, VOset_LR_LO, settings)
%% Retrive locations
% Ground Truth
loc_GT = cat(1, GTset.Views.Location{:});

% Location VO Left-Right Tracking - Local Optimization
loc_VO_LR_LO = cat(1, VOset_LR_LO.VO.Views.Location{:});
if settings.BundleAdjustment
    loc_VO_LR_LO_BA = cat(1, VOset_LR_LO.VO_BA.Views.Location{:});
end

%% All Trajectories
figure(1);
view(0,0)
title(sprintf('Stereo Visual Odometry - KITTI Sequence %s', settings.sequence), 'FontSize', settings.TitleFontSize)
xlabel('X axis - [m]', 'FontSize', settings.LabelFontSize); ylabel('Y axis - [m]', 'FontSize', settings.LabelFontSize); zlabel('Z axis - [m]', 'FontSize', settings.LabelFontSize);
hold on; grid minor;
scatter3(0,0,0, 150, 'sk')
scatter3(loc_GT(:,1), loc_GT(:,2), loc_GT(:,3), 15, 'r', 'filled');
scatter3(loc_VO_LR_LO(:,1), loc_VO_LR_LO(:,2), loc_VO_LR_LO(:,3), 15, 'b', 'filled');
if settings.BundleAdjustment
    scatter3(loc_VO_LR_LO_BA(:,1), loc_VO_LR_LO_BA(:,2), loc_VO_LR_LO_BA(:,3), 15, 'g', 'filled');
    legend('Starting Point','KITTI - GT', 'SVO + LO', 'SVO + BA', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
else
    legend('Starting Point','KITTI - GT', 'SVO + LO', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
end
hold off

%% All Trajectories + Cameras
figure(2)
% Initianl point
plot3(0,0,0, 'sk', 'MarkerSize', 10)
hold on; grid on

% Plot Camera Ground Truth ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Location / Orientation 
plotCamera('Orientation', GTset.Views.Orientation{end,:}', 'Location',  GTset.Views.Location{end,:}, 'Size', settings.CameraSize, 'Color', 'r', 'Label', 'KITTI - GT', 'AxesVisible', true);
% Trajectory
plot3(loc_GT(:,1), loc_GT(:,2), loc_GT(:,3), 'r-', 'MarkerSize', settings.markerLine);

% Plot Camera SVO + LO ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Location / Orientation 
plotCamera('Orientation', VOset_LR_LO.VO.Views.Orientation{end,:}', 'Location', VOset_LR_LO.VO.Views.Location{end,:}, 'Size', settings.CameraSize, 'Color', [0 0.4470 0.7410], 'Label', 'VO + LO', 'AxesVisible', true);
% Trajectory
plot3(loc_VO_LR_LO(:,1), loc_VO_LR_LO(:,2), loc_VO_LR_LO(:,3), 'Color', 'b', 'LineStyle', '-', 'LineWidth', settings.LineWidth)

if settings.BundleAdjustment
    % Plot Camera SVO + DT + LO ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Location / Orientation 
    plotCamera('Orientation', VOset_LR_LO.VO_BA.Views.Orientation{end,:}', 'Location', VOset_LR_LO.VO_BA.Views.Location{end,:}, 'Size', settings.CameraSize, 'Color', [0.8500 0.3250 0.0980], 'Label', 'SVO + DT + LO + BA', 'AxesVisible', true);
    % Trajectory
    plot3(loc_VO_LR_LO_BA(:,1), loc_VO_LR_LO_BA(:,2), loc_VO_LR_LO_BA(:,3), 'Color', 'g', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
end

% Set Y-axis to be vertical pointing down.
view(gca, 3);
set(gca, 'CameraUpVector', [0, -1, 0]);
camorbit(gca, -120, 0, 'data', [0, 1, 0]);
% Title and Labels
xlabel('X axis - [m]', 'FontSize', settings.LabelFontSize); ylabel('Y axis - [m]', 'FontSize', settings.LabelFontSize); zlabel('Z axis - [m]', 'FontSize', settings.LabelFontSize); 
title(sprintf('Camera Trajectory - Stereo Visual Odometry - KITTI Sequence %s', settings.sequence), 'FontSize', settings.TitleFontSize);
legend('Starting Point', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
set(gca,'FontSize',14)
hold off


%% All ABSOLUTE Errors in XYZ
% X-axis limits
limits(1,1) = max(abs(VOset_LR_LO.T_ABSerror_VO(:,1)));
if settings.BundleAdjustment
    limits(2,1) = max(abs(VOset_LR_LO.T_ABSerror_VO_BA(:,1)));
end

% Y-axis limits
limits(1,2) = max(abs(VOset_LR_LO.T_ABSerror_VO(:,2)));
if settings.BundleAdjustment
    limits(2,2) = max(abs(VOset_LR_LO.T_ABSerror_VO_BA(:,2)));
end

% Z-axis limits
limits(1,3) = max(abs(VOset_LR_LO.T_ABSerror_VO(:,3)));
if settings.BundleAdjustment
    limits(2,3) = max(abs(VOset_LR_LO.T_ABSerror_VO_BA(:,3)));
end

figure(3)
% X-axis plot
subplot(3,1,1)
title(sprintf('RMS Error SVO vs KITTI GT - Sequence %s', settings.sequence), 'FontSize', settings.TitleFontSize)
hold on; grid minor;
xlabel('Frame', 'FontSize', settings.LabelFontSize); ylabel('X axis - [m]', 'FontSize', settings.LabelFontSize); 
plot(VOset_LR_LO.T_ABSerror_VO(:,1), 'Color', 'b', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
if settings.BundleAdjustment
    plot(VOset_LR_LO.T_ABSerror_VO_BA(:,1), 'Color', 'g', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
    legend('SVO + LO', 'SVO + BA', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
else
    legend('SVO + LO', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
end
ylim([0 ceil(max(max(limits)))])
set(gca,'FontSize',14)

% Y-axis plot
subplot(3,1,2)
xlabel('Frame', 'FontSize', settings.LabelFontSize); ylabel('Y axis - [m]', 'FontSize', settings.LabelFontSize); 
hold on; grid minor; 
plot(VOset_LR_LO.T_ABSerror_VO(:,2), 'Color', 'b', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
if settings.BundleAdjustment
    plot(VOset_LR_LO.T_ABSerror_VO_BA(:,2), 'Color', 'g', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
    legend('SVO + LO', 'SVO + BA', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
else
    legend('SVO + LO', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
end
ylim([0 ceil(max(max(limits)))])
set(gca,'FontSize',14)

% Z-axis plot
subplot(3,1,3)
xlabel('Frame', 'FontSize', settings.LabelFontSize); ylabel('Z axis - [m]', 'FontSize', settings.LabelFontSize); 
hold on; grid minor;
plot(VOset_LR_LO.T_ABSerror_VO(:,3), 'Color', 'b', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
if settings.BundleAdjustment
    plot(VOset_LR_LO.T_ABSerror_VO_BA(:,3), 'Color', 'g', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
    legend('SVO + LO', 'SVO + BA', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
else
    legend('SVO + LO', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
end
ylim([0 ceil(max(max(limits)))])
set(gca,'FontSize',14)



%% Translationlan error VO vs GT
figure(4)
title(sprintf('Translational RMS Error SVO vs KITTI GT - Sequence %s', settings.sequence), 'FontSize', settings.TitleFontSize)
xlabel('Frame', 'FontSize', settings.LabelFontSize); ylabel('Translational RMS Error - [m]', 'FontSize', settings.LabelFontSize); 
hold on; grid minor;
plot(VOset_LR_LO.TR_error_VO.T_error, 'Color', 'b', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
if settings.BundleAdjustment
    plot(VOset_LR_LO.TR_error_VO_BA.T_error, 'Color', 'g', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
    legend('SVO + LO', 'SVO + BA', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
else
    %ylim([0 65])
    legend('SVO + LO', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
end
set(gca,'FontSize',14)

%% Rotational error VO vs GT
figure(5)
title(sprintf('Rotational RMS Error SVO vs KITTI GT - Sequence %s', settings.sequence), 'FontSize', settings.TitleFontSize)
xlabel('Frame', 'FontSize', settings.LabelFontSize); ylabel('Rotational RMS Error - [deg]', 'FontSize', settings.LabelFontSize); 
hold on; grid minor;
plot(VOset_LR_LO.TR_error_VO.R_error, 'Color', 'b', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
if settings.BundleAdjustment
    plot(VOset_LR_LO.TR_error_VO_BA.R_error, 'Color', 'g', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
    legend('SVO + LO', 'SVO + BA', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
else
    legend('SVO + LO', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
end
set(gca,'FontSize',14)


%% Velocity graph
figure(6)
title(sprintf('Estimated Velocity SVO vs KITTI GT - Sequence %s', settings.sequence), 'FontSize', settings.TitleFontSize)
xlabel('Frame', 'FontSize', settings.LabelFontSize); ylabel('Velocity - [km/h]', 'FontSize', settings.LabelFontSize); 
hold on; grid minor;
plot(VOset_LR_LO.Odometry.GT.velocityVector, '-r', 'LineWidth', settings.LineWidth)
plot(VOset_LR_LO.Odometry.VO.velocityVector, 'Color', 'b', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
if settings.BundleAdjustment
    plot(VOset_LR_LO.OdometryBA.VO.velocityVector, 'Color', 'g', 'LineStyle', '-', 'LineWidth', settings.LineWidth)
    legend('SVO + LO', 'SVO + BA', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
else
    legend('SVO + LO', 'FontSize',settings.LegendFontSize, 'Location', settings.LegendLocation)
end
set(gca,'FontSize',14)
end

function [T_error, T_error_ABS] = getTranslationError(GTset, VOset)
T_error = zeros();
T_error_ABS = zeros();

for i = 1 : length(GTset.Views.ViewId)
    for j = 1 : 3
        loc_GT = GTset.Views.Location{i};
        loc_VO = VOset.Views.Location{i};
        
        T_error(i,j) = loc_GT(:,j) - loc_VO(:,j);
        T_error_ABS(i,j) = sqrt((loc_GT(:,j) - loc_VO(:,j))^2);
    end
end

end

function [error] = getTranslationRotationError(GTset, VOset)

T_error = zeros();
R_error = zeros();

% Get translation error VO vs GT
for i = 1 : length(GTset.Views.ViewId)
    % Translation error
    GT_T = GTset.Views.Location{i};
    VO_T = VOset.Views.Location{i};
    T_error(i,1) = sqrt((GT_T(1)-VO_T(1))^2 + (GT_T(2)-VO_T(2))^2 + (GT_T(3)-VO_T(3))^2);
    
    % Rotation Error
    GT_R = rotationMatrixToVector(GTset.Views.Orientation{i});
    VO_R = rotationMatrixToVector(VOset.Views.Orientation{i});
    R_error(i,1) = sqrt((GT_R(1)-VO_R(1))^2 + (GT_R(2)-VO_R(2))^2 + (GT_R(3)-VO_R(3))^2);
end

error.T_error = T_error;
error.R_error = R_error;
error.Mean_T_error = mean(T_error);
error.Mean_R_error = mean(R_error);
end

function [Odometry] = getEstimatedOdomery(GTset, VOset)
Frequency = 10;  % 10 Hz or 10 frames per second
deltaTime = 1 / Frequency;

loc_GT = cat(1, GTset.Views.Location{:});
loc_VO = cat(1, VOset.Views.Location{:});

distance_GT(1) = norm(loc_GT(1));
distance_VO(1) = norm(loc_VO(1));

for i = 2 : length(GTset.Views.ViewId)    
    distance_GT(i) = abs(norm(loc_GT(i)) - norm(loc_GT(i-1)));
    distance_VO(i) = abs(norm(loc_VO(i)) - norm(loc_VO(i-1)));
    
    velocity_GT(i) = (abs(distance_GT(i)) / deltaTime) * 3.6;
    velocity_VO(i) = (abs(distance_VO(i)) / deltaTime) * 3.6;    
end

GT.distanceVector = distance_GT;
VO.distanceVector = distance_VO;

GT.velocityVector = velocity_GT;
VO.velocityVector = velocity_VO;

GT.totalDisplacement = sum(distance_GT);
VO.totalDisplacement = sum(distance_VO);

% Min velocity
GT.minVelocity = min(velocity_GT(1,2:end));
VO.minVelocity = min(velocity_VO(1,2:end));

% Max Velocity
GT.maxVelocity = max(velocity_GT);
VO.maxVelocity = max(velocity_VO);

% Average Velocity
GT.meanVelocity = mean(velocity_GT);
VO.meanVelocity = mean(velocity_VO);

Odometry.GT = GT;
Odometry.VO = VO;
end