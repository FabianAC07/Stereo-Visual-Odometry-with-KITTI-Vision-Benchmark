%% updatePlotting ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This function updates the general plot during the SVO pipeline

    * I/O       * Objects       * Description   
    Inputs:     - viewId        - Current frame index
                - SVO           - Struct object which contains the data of
                                  the Stereo Visual Odometry pipeline
                - currState     - Struc containing current state data
                - dataset       - Struct containing the stereo parameters
                                  of the stereo camera
                - plotting      - Object to store all the plotting 
                                  parameters

    Outputs:    - plotting      - Object to store all the plotting 
                                  parameters

    Subfunctions: N/A

    Created by: Fabian Aguilar.
    Date:       02/25/19
    Edition:    3
    Edit Date:  08/03/20
%}

%% Start Funtion ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function plotting = updatePlotting(viewId, SVO, currState, dataset,...
                    plotting)
                
run('parameters.m')

%% Update 3D Plot
% Update Location
plotting.Camera_PE.Location = SVO.vVOset.Views.Location{viewId};%.* [1 -1 1];
plotting.Camera_GT.Location = dataset.Views.Location{viewId};

% Update Orientation
plotting.Camera_PE.Orientation = SVO.vVOset.Views.Orientation{viewId}';
plotting.Camera_GT.Orientation = dataset.Views.Orientation{viewId}';

% Update Trajectories
loc_PE = cat(1, SVO.vVOset.Views.Location{:});%.* [1 -1 1];
loc_GT = cat(1, dataset.Views.Location{1:viewId,:});

set(plotting.Trajectory_PE, 'XData', loc_PE(:,1), 'YData', loc_PE(:,2), ...
    'ZData', loc_PE(:,3));

set(plotting.Trajectory_GT, 'XData', loc_GT(:,1), 'YData', loc_GT(:,2), ...
    'ZData', loc_GT(:,3));

if params.plots.plotLandmarks
    landmarks = SVO.landmarks; %.* [-1 1 1];
    set(plotting.Landmarks, 'XData', landmarks(:,1), 'YData', ...
        landmarks(:,2), 'ZData', landmarks(:,3));
end

%% Update 2D Top View
% subplot(3,3,[2,5])
set(plotting.TopView_PE, 'XData', loc_PE(1:viewId,1), 'YData', loc_PE(1:viewId,2), 'ZData', loc_PE(1:viewId,3));
set(plotting.TopView_GT, 'XData', loc_GT(1:viewId,1), 'YData', loc_GT(1:viewId,2), 'ZData', loc_GT(1:viewId,3));
if params.plots.plotLandmarks
    set(plotting.TopView_LM, 'XData', landmarks(:,1), 'YData', landmarks(:,2), 'ZData', landmarks(:,3));
end

%% Update Error plots
set(plotting.error.X, 'YData', SVO.error.X(:,1));
set(plotting.error.Y, 'YData', SVO.error.Y(:,1));
set(plotting.error.Z, 'YData', SVO.error.Z(:,1));

%% Update Inliers plot viewId to viewId
subplot(3,3,[7,8])
imshow(currState.Frames.Left);
hold on, axis on;
title(sprintf('Frame: %d \n Number of Landmarks added to Sparse Point Cloud: %d ', viewId, length(currState.landmarks)));
% scatter(currState.keypoints.Location(:,1), currState.keypoints.Location(:,2), 5, 'blue', 'filled', 'Marker', 'o');
scatter(currState.keypoints(:,1), currState.keypoints(:,2), 5, 'red', 'filled', 'Marker', 'o');
% scatter(currState.tracked_P3P.Location(:,1), currState.tracked_P3P.Location(:,2), 5, 'red', 'filled', 'Marker', '*');
legend( 'Keypoints')
hold off

% Add BA to plots
if params.BA.activate 
    plotting.Camera_BA.Location = SVO.vVOsetBA.Views.Location{viewId};
    plotting.Camera_BA.Orientation = SVO.vVOsetBA.Views.Orientation{viewId}';
    loc_BA = cat(1, SVO.vVOsetBA.Views.Location{:});
    set(plotting.Trajectory_BA, 'XData', loc_BA(:,1), 'YData', loc_BA(:,2), ...
        'ZData', loc_BA(:,3));
    set(plotting.TopView_BA, 'XData', loc_BA(1:viewId,1), 'YData', ...
        loc_BA(1:viewId,2), 'ZData', loc_BA(1:viewId,3));
    set(plotting.errorBA.X, 'YData', SVO.errorBA.X(:,1));
    set(plotting.errorBA.Y, 'YData', SVO.errorBA.Y(:,1));
    set(plotting.errorBA.Z, 'YData', SVO.errorBA.Z(:,1));
end

end
% End of Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~