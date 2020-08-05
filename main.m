%% Stereo Visual Odometry 
%{

This software is the main code for the Master Thesis Project "Sparse Visual
Odometry with Local Non-Linear Least-Squares Optimization for Navigation of
Autonomous Vehicles" [1].

This code is a stereo vision pipeline which uses the KITTI dataset [2] as
input data. 
 
For further reading, please refer to [1].

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

% Run params 
run('parameters.m');

diary(params.save.commandwindow)
diary on
fprintf('Stereo Visual Odometry Process Has Started... \n')

%% Setup and loading of dataset properties ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% automatically cd to /src directory
cd(fileparts(which('src/main.m')));

% Add paths
addpath(genpath('functions'))

% Get KITTI DataSet
fprintf('Loading KITTI Camera Parameters and Dataset... \n')
dataset = getKITTI(params.data);

% Create initProcess contditions and object to store values:
SVO.vVOset = viewSet;       % View Set with Vision Odometry data [R,T]
SVO.vVOsetBA = viewSet;     % View Set for VO data with Boundle Adjusment
SVO.landmarks = [];         % 3D pointcloud for mapping (N,3) matrix
SVO.keypoints = [];         % 2D image points
SVO.poseLandmarks = [];     %
SVO.error = [];             % Error

% In case the process needs to restart
if params.process.restart.do
    global restart;
    restart = false;
end

%% ======================================================================= 
%  Initail Stage ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%  =======================================================================
fprintf('Starting Stereo Visual Odometry Process... \n')
for frame = 1 : params.initial.loop.numTrials
    % Errase anywhing inside of the SVO.vVOset and SVO.vVOsetBA
    SVO.vVOset = viewSet;
    SVO.vVOsetBA = viewSet;
    
    % Run initProcess
    [currState, prevState, SVO, viewId] = getInitialProcessing(SVO, dataset);
    
    if currState.InliersFilter.ratio > params.initial.loop.inlierRatio
        fprintf('Inlier ratio: %0.3f is higher than Treshold: %0.3f\n', currState.InliersFilter.ratio, params.initial.loop.inlierRatio)
            if params.video.flag
                % Display Frames
                close all;
                step(params.video.player, currState.Frames.Left)
                
                if params.video.record
                    % Start Wirting the Output Video
                    open(params.video.output);
                    writeVideo(params.video.output, currState.Frames.Left);
                end
            end
        break
    elseif frame == params.initial.loop.numTrials
        close all;
        warning('\n\n Start Process FAILED, bad value in getInitialProcess() after max iterations\n\n')
    end
    close all;
    fprintf('\n\n Start Process FAILED, run again\n\n'); 
end
fprintf('Initial Stage of Stereo Visual Odometry Process is DONE... \n')

% Plotting Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if params.plots.plotFlag
    plotting = setupPlotting(SVO, dataset.vGTset);
end

% Update Plot at second instant of time
if params.plots.plotFlag
        plotting = updatePlotting(viewId, SVO, currState, ...
                                  dataset.vGTset, plotting);
end

%% =======================================================================
%  Contineous Processing ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%  =======================================================================
% Contineous algorithm from second instant of time until the last frame
fprintf('Continuous Stereo Visual Odometry Process... \n')
for frame = 3 : length(dataset.img_dir_L) - 1
    fprintf('============================================================= \n')
    fprintf('Processing Frame:              %d \n', frame);
    tic
    % Current State is the input previus State
    prevState = currState;
    
    switch(params.process.restart.do)
        case false
            % Frame processing ......
            [currState, SVO, viewId] = getContinuousProcessing(viewId+1, prevState, SVO, dataset);
        case true
            if ~restart
                % Frame processing ......
                [currState, SVO, viewId] = getContinuousProcessing(viewId+1, prevState, SVO, dataset);

            else 
                % Do Restart Process .......
                [currState, SVO, viewId] = getRestartProcess(viewId+1, SVO, dataset);
                restart = false;
            end
    end
    time = toc;
    
    % Plot results
    if params.plots.plotFlag
        plotting = updatePlotting(viewId, SVO, currState, ...
                                  dataset.vGTset, plotting);
    end
    pause(0.01)
    fprintf('Time of processing:            %3.3f \n', time);
    fprintf('============================================================= \n')
    
    if params.video.flag
        % Display Frames
        step(params.video.player, currState.Frames.Left)
        if params.video.record
            % Write the Output Video
            writeVideo(params.video.output, currState.Frames.Left);
        end
    end
end
fprintf('Stereo Visual Odometry Process Has Finished... \n')

%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% Save workspace for further reference... 
if params.save.save
    if ~exist(params.save.folder, 'dir')
        fprintf('Results folder does not exist... \n')
        fprintf('Creating /results folder... \n')
        mkdir(params.save.folder)
    end
    fprintf('Saving workspace... \n')
    save(params.save.workspace)
    movefile(params.save.workspace, params.save.folder)
    fprintf('Workspace has been saved at /results folder succesfully... \n')
end

% Close Output Video
if params.video.record
    close(params.video.output)
    if ~exist(params.save.folder, 'dir')
        fprintf('Results folder does not exist... \n')
        fprintf('Creating /results folder... \n')
        mkdir(params.save.folder)
    end
    movefile(params.video.filename, params.save.folder)
    fprintf('Output Video has been saved at /results folder succesfully... \n')
end

diary off
movefile(params.save.commandwindow, params.save.folder)
% End of main script ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

