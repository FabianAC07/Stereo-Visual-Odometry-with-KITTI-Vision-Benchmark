%% Stereo Visual Odometry Parameters 
%{

Parameters file to process Stereo Visual Odometry Software

Author:         Fabian Aguilar
Date:           01/27/19
Edition:        3
Latest Edition: 08/03/20
%}
%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Select dataset to work with:
% TODO: Include other datasets such as MALAGA

% Options available
% 'KITTI'
params.dataset = 'KITTI';

switch(params.dataset)
    case 'KITTI'
        %% KITTI Dataset ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        % Load the dataset and sequence to use (Currently KITTI DataSet)
        % 0 = Sequence 00       4541 images
        % 1 = Sequence 01       1101 images
        % 2 = Sequence 02       4661 images
        % 3 = Sequence 03       801 images
        % 4 = Sequence 04       271 images
        % 5 = Sequence 05       2761 images
        % 6 = Sequence 06       1101 images
        % 7 = Sequence 07       1101 images
        % 8 = Sequence 08       4071 images
        % 9 = Sequence 09       1591 images
        % 10 = Sequence 10      1201 images

        params.data.Sequence = 03;
        
        % Load Grount Truth: 0 = NO, 1 = YES
        params.data.GroundTruth = true;
        
        % Frequency of the images 10 Hz or 10 frames per second
        params.data.Frequency = 10;
end

%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
params.save.save = true;
params.save.folder = 'results';
params.save.workspace = 'KITTI_Seq_03.mat';
params.save.commandwindow = 'KITTI_Seq_03.txt';

%% Debugging ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
params.debugging.plotting = false;
params.debugging.checkFeatures = false;

%% Video Settings ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
params.video.flag = true;
% Initialize the video output
params.video.player = vision.DeployableVideoPlayer('Location', [10, 50], ...
                'Size', 'Custom' , 'CustomSize', [800, 400]);
params.video.record = false;
% Initialize the Video Wirter
params.video.filename = 'KITTI_Dataset_Seq_03.avi';
params.video.output = VideoWriter(params.video.filename);


%% Plotting ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
params.plots.plotFlag = true;
params.plots.plotLandmarks = true;
params.plots.camSize = 2;

%% Initial Process ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Calculate the first stage of the process by processing the fisrt 2
% instants of time in the image or video sequence

params.initial.loop.numTrials = 1;                  % Number of trials
params.initial.location = zeros(1,3);               % First Location
params.initial.orientation = eye(3);                % First Orientation
params.initial.loop.inlierRatio = 0.5;              % Inlier Ratio for start loop

%% Continuos Processing ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% Restart Process Threshold
params.process.restart.do = false;
params.process.restart.triggerDistance = 50;  % meters
params.process.restart.triggerKeypoints = 20; % Keypoints
params.process.restart.step = 2;              % Frames


%% Triangulation Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
params.traing.maxRadius = 30;
params.traing.minDistThresh = 2;
params.traing.repErrThresh = 0.5;

%% OpenCV Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Features and Descriptors
% Options: HARRIS, SURF, ORB
% NOTE: GFTT uses Harris Corner Detector
params.opencv.features = 'SURF';

% ORB GFTT Parameters
params.opencv.GFTT.MaxCorners = 5000;               % Sample size
params.opencv.GFTT.QualityLevel = 1e-5;             
params.opencv.GFTT.MinDistance = 2;
params.opencv.GFTT.BlockSize = 3;
params.opencv.GFTT.UseHarrisDetector = true;
params.opencv.GFTT.BriefDescriptorExtractor = 'BriefDescriptorExtractor';
params.opencv.GFTT.Bytes = 64;
params.opencv.GFTT.UseOrientation = false;

% ORB SURF Parameters
params.opencv.SURF.Extended = true;                 % Length of feature vector (64 | 128)
params.opencv.SURF.HessianThreshold = 0.001;        % TUNE this value
params.opencv.SURF.NOctaves = 3;                    % Value MATLAB 
params.opencv.SURF.NOctaveLayers = 4;               % TUNE this value
params.opencv.SURF.Upright = false;                 % Rotation invariance flag (true or false)
params.opencv.SURF.runByKeypointSize = 35; 
params.opencv.SURF.sample = 5000;
params.opencv.SURF.MaxRatio = 0.7;                  % Ratio threshold (0,1] before 0.9

% ORB SURF Parameters
params.opencv.ORB.MaxFeatures = 5000;
params.opencv.ORB.ScaleFactor = 1.2;
params.opencv.ORB.NLevels = 8;
params.opencv.ORB.EdgeThreshold = 31;
params.opencv.ORB.FirstLevel = 0;
params.opencv.ORB.WTA_K = 2;
params.opencv.ORB.ScoreType = 'FAST';
params.opencv.ORB.PatchSize = 31;
params.opencv.ORB.FastThreshold = 10;

% Descriptor for the connection between frames
params.opencv.descriptor.matcher.BA = 'BruteForce';

% Lucas-Kanade Algorithm options
params.opencv.lk_params = {'WinSize',[21 21], 'MaxLevel',4, 'Criteria', ...
                          struct('type','Count+EPS', 'maxCount',30, 'epsilon',0.03), ...
                          'MinEigThreshold',0.001};

% PnP Solver
params.opencv.PnP.numTrials = 5;                    % Num of trials
params.opencv.PnP.deltaT = 1.2;                     % Max translation allowed
params.opencv.PnP.minRatio = 0.5;                   % Min ratio
params.opencv.PnP.IterationsCount = 2000;           % Max num of iterations
params.opencv.PnP.ReprojectionError = 1;            % Max 1 pixel
params.opencv.PnP.Confidence = 0.9999;              % Confidence in results
params.opencv.PnP.Method = 'P3P';                   % P3P solver
                           
% 5-Point Algorithm - RANSAC (Essential Matrix Calculation) 
params.opencv.eEm.numTrials = 5;                    % Number of trials
params.opencv.eEm.inlierRatio = 0.5;                % Min inlier ratio
params.opencv.eEm.Method = 'Ransac';                % Method RANSAC
params.opencv.eEm.Confidence = 0.9999;              % Confidence
params.opencv.eEm.Threshold = 0.8;                  % Threshold

%% Bundle Adjustment ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
params.BA.activate = false;                      
params.BA.PointsUndistorted = true;                 % Points with no distortion
params.BA.AbsoluteTolerance = 1e-9;
params.BA.RelativeTolerance = 1e-9;                 
params.BA.MaxIterations = 500;                      % Max iterations
params.BA.window = 10;                              % frames

% Get Close points ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
params.getClose.treshold = 10;                      % meters



