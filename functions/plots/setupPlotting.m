%% setupPlotting ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This function calculate the first state of the SVO algorithm

    * I/O       * Objects       * Description   
    Inputs:     - SVO           - Struct object which contains the data of
                                  the Stereo Visual Odometry pipeline
                - GrountTruth   - Struct containing the Grount Truth data

    Outputs:    - plotting      - Object to store all the plotting 
                                  parameters

    Subfunctions: N/A

    Created by: Fabian Aguilar.
    Date:       02/25/19
    Edition:    3
    Edit Date:  08/03/20
%}

%% Start Funtion ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function plotting = setupPlotting(SVO, GrountTruth)

close all;

% Run Parameters 
run('parameters.m');

figure(1);   
% Plot GT data vs Pose Estimation on 3D
ax1 = subplot(3,3,[1,4]);

% Set Y-axis to be vertical pointing down.
view(gca, 3);
set(gca, 'CameraUpVector', [0, -1, 0]);
camorbit(gca, -120, 0, 'data', [0, 1, 0]);
hold on; grid on

% Title and Labels
xlabel('X axis - [m]'); ylabel('Y axis - [m]'); zlabel('Z axis - [m]'); 

% Plot Camera Grount Truth ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Location / Orientation 
plotting.Camera_GT = plotCamera('Orientation', GrountTruth.Views.Orientation{1}', ...
    'Location', GrountTruth.Views.Location{1}, 'Size', params.plots.camSize, ...
    'Color', 'r', 'Label', 'Ground Truth', 'AxesVisible', true);
% Trajectory
loc_GT = GrountTruth.Views.Location{1};
plotting.Trajectory_GT = plot3(loc_GT(1), loc_GT(2), loc_GT(3), 'r-');

% Plot Camera Pose Estimation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Reflect Location 
loc_PE = SVO.vVOset.Views.Location{1};% .* [-1 -1 1];
plotting.Camera_PE = plotCamera('Orientation', SVO.vVOset.Views.Orientation{1}, ...
    'Location', loc_PE, 'Size', params.plots.camSize, 'Color', 'b', ...
    'Label', 'SVO', 'AxesVisible', true);
% Trajectory
plotting.Trajectory_PE = plot3(loc_PE(1), loc_PE(2), loc_PE(3), 'b-');

% Plot Landmarks ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if params.plots.plotLandmarks
    landmarks = SVO.landmarks;% .* [-1 -1 1];
    plotting.Landmarks = scatter3(landmarks(:,1), landmarks(:,2), landmarks(:,3), ...
        5,'black','filled','Marker','o', 'MarkerFaceAlpha',.1,'MarkerEdgeAlpha',.1);
end

% Pose Estimation with windows BA
if params.BA.activate
    % Plot Camera Pose Estimation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Reflect Location 
    loc_BA = SVO.vVOsetBA.Views.Location{1};% .* [-1 -1 1];
    plotting.Camera_BA = plotCamera('Orientation', SVO.vVOsetBA.Views.Orientation{1}, ...
        'Location', loc_BA, 'Size', params.plots.camSize, 'Color', 'g', ...
        'Label', 'Pose Estimation', 'AxesVisible', true);
    % Trajectory
    plotting.Trajectory_BA = plot3(loc_BA(1), loc_BA(2), loc_BA(3), 'g-');
    
    %Legend
    legend('Ground Truth','SVO', 'Landmarks', 'SVO with BA', 'Location', 'northwest')
else
    legend( 'Ground Truth','SVO', 'Landmarks', 'Location', 'northwest')
end
title('3D Camera Plot - SVO vs Ground Truth Trajectory');
hold off

%% Top View PE vs GT
ax2 = subplot(3,3,[2,5]);
view(0,0);
% Title and labels
title('Top View - SVO vs Ground Truth Trajectory');
xlabel('X axis - [m]'); ylabel('Y axis - [m]'); zlabel('Z axis - [m]');
hold on; grid minor;
plotting.TopView_GT = scatter3(loc_GT(1), loc_GT(2), loc_GT(3), 15, 'r', 'filled');
plotting.TopView_PE = scatter3(loc_PE(1), loc_PE(2), loc_PE(3), 15, 'b', 'filled');
if params.plots.plotLandmarks
    plotting.TopView_LM = scatter3(landmarks(:,1), landmarks(:,2), landmarks(:,3),...
        5,'black','filled','Marker','o', 'MarkerFaceAlpha',.1,'MarkerEdgeAlpha',.1);
end
if params.BA.activate
    plotting.TopView_BA = scatter3(loc_BA(1), loc_BA(2), loc_BA(3), 15, 'g', 'filled');
    legend('Ground Truth','SVO', 'Landmarks', 'SVO with BA', 'Location', 'northwest')
else
    legend( 'Ground Truth','SVO', 'Landmarks', 'Location', 'northwest')
end
% axis([loc_GT(1)-50, loc_GT(1)+50, loc_GT(2)-50, loc_GT(2)+50 loc_GT(3)-50, loc_GT(3)+50 ]);
hold off

%% Errors vs GT
ax3 = subplot(3,3,3);
plotting.error.X = plot(SVO.error.X(:,1), '--b');
title('Absolute Error SVO Trajectory vs Ground Truth')
if params.BA.activate
    hold on
    plotting.errorBA.X = plot(SVO.errorBA.X(:,1), '--g');
    legend('SVO Error', 'SVO with BA Error', 'Location', 'northwest')
else
    legend('SVO Error', 'Location', 'northwest')
end
xlabel('Frame'); ylabel('X axis - [m]'); 
grid minor

ax4 = subplot(3,3,6);
plotting.error.Y = plot(SVO.error.Y(:,1), '--b');
if params.BA.activate
    hold on
    plotting.errorBA.Y = plot(SVO.errorBA.Y(:,1), '--g');
    legend('SVO Error', 'SVO with BA Error', 'Location', 'northwest')
else
    legend('SVO Error', 'Location', 'northwest')
end
xlabel('Frame'); ylabel('Y axis - [m]'); 
grid minor

ax5 = subplot(3,3,9);
plotting.error.Z = plot(SVO.error.Z(:,1), '--b');
if params.BA.activate
    hold on
    plotting.errorBA.Z = plot(SVO.errorBA.Z(:,1), '--g');
    legend('SVO Error', 'SVO with BA Error', 'Location', 'northwest')
else
    legend('SVO Error', 'Location', 'northwest')
end
xlabel('Frame'); ylabel('Z axis - [m]');
grid minor

plotting.ax1 = ax1;
plotting.ax2 = ax2;
plotting.ax3 = ax3;
plotting.ax4 = ax4;
plotting.ax5 = ax5;

end
% End of Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~