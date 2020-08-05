%% getRestartProcess ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This function restarts the process in case there are not enough
    keypoints to process a frame in getInitialProcessing()

    * I/O       * Objects       * Description   
    Inputs:     - viewId        - ViewSet object counter     
                - SVO           - SVO object to store Location and Mapping 
                - dataset       - dataset object 

    Outputs:    - currState     - Current state object             
                - SVO           - SVO object to updated VO data
                - viewId        - ViewSet object counter

    Subfunctions:
    - getMatchingPoints()
    - cv.findEssentialMat()
    - getDLT()
    - getFilteredTrinagulation()
    - cv.calcOpticalFlowPyrLK()
    - cv.solvePnPRansac()
    - cv.Rodrigues()
    - cv.DescriptorMatcher()
    

    Created by: Fabian Aguilar.
    Date:       02/24/19
    Edition:    3
    Edit Date:  08/03/20
%}

%% Start Funtion ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [currState, SVO, viewId] = getRestartProcess(viewId, SVO, dataset)
% Run Parameters 
run('parameters.m');

% Set "Initial Conditions"
step = params.process.restart.step;
orientation = SVO.vVOset.Views.Orientation{end-1};
location = SVO.vVOset.Views.Location{end-1};

% If doing Bundle adjustment
if params.BA.activate
    orientationBA = SVO.vVOsetBA.Views.Orientation{end-1};
    locationBA = SVO.vVOsetBA.Views.Location{end-1};
end
%% Set the frame where we are doing the rebootstrap
viewId = viewId - step;

%% =======================================================================
%  This is the processing of the first instant of time ....
%  =======================================================================
%% Read the firts pair of images:
prevState.Frames.Left = imread([dataset.img_dir_L(viewId).folder, '/', dataset.img_dir_L(viewId).name]);
prevState.Frames.Right = imread([dataset.img_dir_R(viewId).folder, '/', dataset.img_dir_R(viewId).name]);

% Get matching points
[prevState.matchedPoints, prevState.keyPoints, prevState.descriptors] = ...
    getMatchingKeyPoints(prevState.Frames, params);

if params.debugging.plotting
    figure(100)
    showMatchedFeatures(prevState.Frames.Left, prevState.Frames.Right, ...
       prevState.matchedPoints.Left,prevState.matchedPoints.Right);
end

%% Remove Outliers using Epipolar Geometry (RANSAC) ~~~~~~~~~~~~~~~~~~~~~~
for i = 1 : params.opencv.eEm.numTrials
    % Estimate Essential Matrix (5-Point Algorithm)                         
    [~, mask] = cv.findEssentialMat(num2cell(prevState.matchedPoints.Left,2), ...
                                    num2cell(prevState.matchedPoints.Right,2), ...
                                    'Method', params.opencv.eEm.Method,...
                                    'Confidence', params.opencv.eEm.Confidence, ...
                                    'Threshold', params.opencv.eEm.Threshold);
    
    % mask values are inliers RANSAC values                       
    mask = logical(mask);
    
    % Ensure we get enough inliers
    ratio = sum(mask) / numel(mask); 
    if(ratio > params.opencv.eEm.inlierRatio)
        fprintf('Iterations 5-Point Algortithm: %d \n', i)
        fprintf('Inlier Ratio:                  %.2f \n', ratio);
        break;
    else
        fprintf('Failed to calulate E, Iter:    %  \n', i)
        fprintf('Inlier Ratio:                  %.2f \n', ratio)
        fprintf('Run again. Iters. left:        %d \n', params.opencv.eEm.numTrials - i)
        if i == params.opencv.eEm.numTrials
            fprintf('Fraction of inliers for E:     %.2f \n', ratio);
            disp('Max iterations in 5-Point Algorithm trials reached, bad E is likely');
        end
    end
end

% Retrive inliers on the first frame
prevState.Inliers.Left = prevState.matchedPoints.Left(mask,:);
prevState.Inliers.Right = prevState.matchedPoints.Right(mask,:);

%% Perform Direct Linear Transformation - Triangulation
fprintf('Calculating World Points .... \n')
[prevState.xyzPoints, reprojError] = getDLT(prevState, dataset);
        
% Filter Triangulated points
[prevState.xyzFilter, idxFilter, ratioFilter] = getFilteredTrinagulation(prevState.xyzPoints, ...
                                reprojError, params.traing.maxRadius, ...
                                params.traing.minDistThresh, ...
                                params.traing.repErrThresh);

% Use index filtered to filter the inliers from RANSAC
prevState.InliersFilter.Left = prevState.Inliers.Left(idxFilter,:);
prevState.InliersFilter.Right = prevState.Inliers.Right(idxFilter,:);
prevState.InliersFilter.ratio = ratioFilter;

% Plot inliers matches on first frame:
if params.debugging.plotting
    figure(101)
    showMatchedFeatures(prevState.Frames.Left, prevState.Frames.Right, ...
        prevState.InliersFilter.Left, prevState.InliersFilter.Right);
    legend('Features Right Camera', 'Features Left Camera')
    title('Feature Detection and Matching')
end

%% Set the first view of the pose estimation .............................       
% Calculate the error compared to Ground Truth
loc_GT = dataset.vGTset.Views.Location{viewId-1};
loc_PE = location; %.* [-1 -1 1];
SVO.error.X(viewId,1) = abs(loc_GT(1) - loc_PE(1));
SVO.error.Y(viewId,1) = abs(loc_GT(2) - loc_PE(2));
SVO.error.Z(viewId,1) = abs(loc_GT(3) - loc_PE(3));

% Print current location
fprintf('Frame processing done .... \n')
fprintf('Ground Truth location:     X = %4.3f, Y = %4.3f, Z = %4.3f \n', ...
        loc_GT(1), loc_GT(2), loc_GT(3))
fprintf('Current location:          X = %4.3f, Y = %4.3f, Z = %4.3f \n', ...
        loc_PE(1), loc_PE(2), loc_PE(3))

% Add view to the set of VO with LSQ only
SVO.vVOset = updateView(SVO.vVOset, viewId, 'Points', cat(1, prevState.keyPoints.keyPointsLeft.pt),...
            'Orientation', orientation, 'Location', location);

if params.BA.activate
    % Add view to the set of VO with LSQ + windowed BA
    SVO.vVOsetBA = updateView(SVO.vVOsetBA, viewId, 'Points', cat(1, prevState.keyPoints.keyPointsLeft.pt), ... 
                'Orientation', orientationBA, 'Location', locationBA);
end

%% =======================================================================
% Process second instant of time ....
% ========================================================================
fprintf('============================================================= \n')
fprintf('Processing Frame 2 .... \n')
viewId = viewId + 1;
%% Read the second instant of time frame
currState.Frames.Left = imread([dataset.img_dir_L(viewId).folder, '/', dataset.img_dir_L(viewId).name]);
currState.Frames.Right = imread([dataset.img_dir_R(viewId).folder, '/', dataset.img_dir_R(viewId).name]);

% Get matching points
[currState.matchedPoints, currState.keyPoints, currState.descriptors] = ...
    getMatchingKeyPoints(currState.Frames, params);

%% Remove Outliers using Epipolar Geometry (RANSAC) ~~~~~~~~~~~~~~~~~~~~~~
for i = 1 : params.opencv.eEm.numTrials
    % Estimate Essential Matrix (5-Point Algorithm)                         
    [~, mask] = cv.findEssentialMat(num2cell(currState.matchedPoints.Left,2), ...
                                    num2cell(currState.matchedPoints.Right,2), ...
                                    'Method', params.opencv.eEm.Method,...
                                    'Confidence', params.opencv.eEm.Confidence, ...
                                    'Threshold', params.opencv.eEm.Threshold);
    
    % mask values are inliers RANSAC values                       
    mask = logical(mask);
    
    % Ensure we get enough inliers
    ratio = sum(mask) / numel(mask); 
    if(ratio > params.opencv.eEm.inlierRatio)
        fprintf('Iterations 5-Point Algortithm: %d \n', i)
        fprintf('Inlier Ratio:                  %.2f \n', ratio);
        break;
    else
        fprintf('Failed to calulate E, Iter:    %  \n', i)
        fprintf('Inlier Ratio:                  %.2f \n', ratio)
        fprintf('Run again. Iters. left:        %d \n', params.opencv.eEm.numTrials - i)
        if i == params.opencv.eEm.numTrials
            fprintf('Fraction of inliers for E:     %.2f \n', ratio);
            disp('Max iterations in 5-Point Algorithm trials reached, bad E is likely');
        end
    end
end

% Retrive inliers on the current frame
currState.Inliers.Left = currState.matchedPoints.Left(mask,:);
currState.Inliers.Right = currState.matchedPoints.Right(mask,:);

%% Perform Direct Linear Transformation - Triangulation
fprintf('Calculating World Points .... \n')
[currState.xyzPoints, reprojError] = getDLT(currState, dataset);
        
% Filter Triangulated points
[currState.xyzFilter, idxFilter, ratioFilter] = getFilteredTrinagulation(currState.xyzPoints, ...
                                reprojError, params.traing.maxRadius, ...
                                params.traing.minDistThresh, ...
                                params.traing.repErrThresh);

% Use index filtered to filter the inliers from RANSAC
currState.InliersFilter.Left = currState.Inliers.Left(idxFilter,:);
currState.InliersFilter.Right = currState.Inliers.Right(idxFilter,:);
currState.InliersFilter.ratio = ratioFilter;

% Plot inliers matches on second frame:
if params.debugging.plotting
    figure(102)
    showMatchedFeatures(currState.Frames.Left, currState.Frames.Right, ...
        currState.InliersFilter.Left, currState.InliersFilter.Right);
    legend('Features Right Camera', 'Features Left Camera')
    title('Feature Detection and Matching')
end

%% =======================================================================
%  Initialize pose estimation of second frame wrp to the first one:
%  =======================================================================
fprintf('Tracking further frame .... \n')

% Tracking Further Left
p0_Left = cellfun(@(tr) tr(end,:), num2cell(prevState.InliersFilter.Left,2)', 'UniformOutput',false);
p1_Left = cv.calcOpticalFlowPyrLK(prevState.Frames.Left, currState.Frames.Left, p0_Left, params.opencv.lk_params{:});
p0r_Left = cv.calcOpticalFlowPyrLK(currState.Frames.Left, prevState.Frames.Left, p1_Left, params.opencv.lk_params{:});
% keep only good matches
idx_LK_Error_Left = cellfun(@(a,b) max(abs(a - b)), p0_Left, p0r_Left) < 1;

% % Tracking Fruther Right
p0_Right = cellfun(@(tr) tr(end,:), num2cell(prevState.InliersFilter.Right,2)', 'UniformOutput',false);
p1_Right = cv.calcOpticalFlowPyrLK(prevState.Frames.Right, currState.Frames.Right, p0_Right, params.opencv.lk_params{:});
p0r_Right = cv.calcOpticalFlowPyrLK(currState.Frames.Right, prevState.Frames.Right, p1_Right, params.opencv.lk_params{:});
% keep only good matches
idx_LK_Error_Right = cellfun(@(a,b) max(abs(a - b)), p0_Right, p0r_Right) < 1;

% Joint index and select only good tracked points
idx_LK = and(idx_LK_Error_Left, idx_LK_Error_Right);
trackedPoints_fur_Left = p1_Left(idx_LK);
trackedPoints_fur_Left = cat(1, trackedPoints_fur_Left{:});
trackedPoints_fur_Rigth = p1_Right(idx_LK);
trackedPoints_fur_Rigth = cat(1, trackedPoints_fur_Rigth{:});

% Remove non-valid points from inliers filtered in prev state
prevState.InliersP3P.Left = prevState.InliersFilter.Left(idx_LK,:);
prevState.InliersP3P.Right = prevState.InliersFilter.Right(idx_LK,:);
prevState.xyzP3P = prevState.xyzFilter(idx_LK,:);

% Remove Outliers using Epipolar Geometry (RANSAC) ~~~~~~~~~~~~~~~~~~~~~~
fprintf('Calculating Essential Matrix - RANSAC for tracked points .... \n') 
for i = 1 : params.opencv.eEm.numTrials
    % Estimate Essential Matrix (5-Point Algorithm)
    [~, mask_Left] = cv.findEssentialMat(num2cell(prevState.InliersP3P.Left, 2), ...
                                     num2cell(trackedPoints_fur_Left, 2), ...
                                     'Method', params.opencv.eEm.Method,...
                                     'Confidence', params.opencv.eEm.Confidence, ...
                                     'Threshold', params.opencv.eEm.Threshold);
                          
    [~, mask_Right] = cv.findEssentialMat(num2cell(prevState.InliersP3P.Right, 2), ...
                                     num2cell(trackedPoints_fur_Rigth, 2), ...
                                     'Method', params.opencv.eEm.Method,...
                                     'Confidence', params.opencv.eEm.Confidence, ...
                                     'Threshold', params.opencv.eEm.Threshold);
    
    % Joint Inliers
    mask = logical(and(mask_Left, mask_Right));
    
    % Ensure we get enough inliers
    ratio = sum(mask) / numel(mask);  
    if(ratio > params.opencv.eEm.inlierRatio)
        fprintf('Iterations 5-Point Algortithm: %d \n', i)
        fprintf('Inlier Ratio:                  %.2f \n', ratio);
        break;
    else
        fprintf('Failed to calulate E, Iter:    %  \n', i)
        fprintf('Inlier Ratio:                  %.2f \n', ratio)
        fprintf('Run again. Iters. left:        %d \n', params.opencv.eEm.numTrials - i)
        if i == params.opencv.eEm.numTrials
            fprintf('Fraction of inliers for E:     %.2f \n', ratio);
            disp('Max iterations in 5-Point Algorithm trials reached, bad E is likely');
        end
    end
end

% Remove Outliers from tracked points correspondances 
prevState.InliersP3P.Left = prevState.InliersP3P.Left(mask,:);
prevState.InliersP3P.Right = prevState.InliersP3P.Right(mask,:);
prevState.xyzP3P = prevState.xyzP3P(mask,:);
currState.tracked_P3P_Left = trackedPoints_fur_Left(mask,:);
currState.tracked_P3P_Right = trackedPoints_fur_Rigth(mask,:);

if params.debugging.plotting
    figure(103)
    showMatchedFeatures(prevState.Frames.Left, currState.Frames.Left, ...
        prevState.InliersP3P.Left, currState.tracked_P3P_Left);
    legend('Features @ T_k', 'Features @ T_k-1')
    title('Feature Tracking over time')
end

fprintf('Frame processing done .... \n')
fprintf('============================================================= \n')

%% =======================================================================
%  Run P3P solver to determine pose estimation ....
%  =======================================================================

fprintf('Processing Pose Estimation - P3P solver .... \n')

%% Run Pose Estimation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
for i = 1 : params.opencv.PnP.numTrials
    % Run P3P and RANSAC
    [rvec, tvec, success, idxPose] = cv.solvePnPRansac(num2cell(prevState.xyzP3P, 2),...
                num2cell(currState.tracked_P3P_Left,2), dataset.PL(:, 1:3), ...
                'IterationsCount', params.opencv.PnP.IterationsCount, ...
                'ReprojectionError', params.opencv.PnP.ReprojectionError, ...
                'Confidence', params.opencv.PnP.Confidence, ...
                'Method', params.opencv.PnP.Method);
    
    % Decompose matrix and translation vector
    [rmat,~] = cv.Rodrigues(rvec);
    tvec = (-rmat' * tvec)';

    % Ensure we get enough inliers
    ratio = numel(idxPose) / numel(prevState.xyzP3P(:, 1)); 
    if(norm(tvec) < params.opencv.PnP.deltaT) && ratio > params.opencv.PnP.minRatio && success % Check if position delta is too large
        fprintf('Norm of Translation:           %2.2f \n', norm(tvec));
        fprintf('P3P Iterations:                %d \n', i);
        fprintf('Ratio of Inliers:              %.2f \n', ratio);
        break;  
    elseif i == params.opencv.PnP.numTrials
        warning('Max iter. i==%d reached, still large position delta produced!', i)
        %TODO: Skip frame in case a bad calculation of location is given
        %TODO: Check that the rotation matrix is good compared to GT        
    else
        warning('Large position delta produced after %d iterations!', i)
    end
end

%% =======================================================================
%  Non-Linear Optimization of R and T from P3P Solver....
%  =======================================================================
fprintf('Processing Pose Estimation Optimization - LSQ solver .... \n')
% Run Least Squares non linear optimization to optimize R T from P3P solver
% Check c++ index
if idxPose(1) == 0
    idxPose(1) = [];
end
                 
[R, T] = getOptimization(rmat', tvec, idxPose, prevState.xyzP3P, ...
                         currState.tracked_P3P_Left, dataset.PL', ...
                         currState.tracked_P3P_Right, dataset.PR');

currState.Orientation = SVO.vVOset.Views.Orientation{end} * R;
currState.Location = SVO.vVOset.Views.Location{end} + (SVO.vVOset.Views.Orientation{end} * T')';    

%% =======================================================================
%  Add Orientation / Location for VO ....
%  =======================================================================
% Add view to SLAM (Orientation / Location / Inliers)
SVO.vVOset = updateView(SVO.vVOset, viewId, 'Points', cat(1, currState.keyPoints.keyPointsLeft.pt), ...
            'Orientation', currState.Orientation, 'Location', currState.Location); 

% Feature Connection for Bundle Adjusment
matcher = cv.DescriptorMatcher(params.opencv.descriptor.matcher.BA);
matches = matcher.knnMatch(prevState.descriptors.descriptorsLeft,...
                           currState.descriptors.descriptorsLeft, 2);

idx = cellfun(@(m) m(1).distance < 0.75 * m(2).distance, matches);
matches = cellfun(@(m) m(1), matches(idx));

idxMatch = uint32([cat(1, matches.queryIdx), cat(1, matches.trainIdx)]);

if any(idxMatch(:,1) < 1) || any(idxMatch(:,2) < 1)
    [row, ~] = find(idxMatch < 1);
    idxMatch(row,:) = [];
end
         
% Add the conection between both intstans of time
SVO.vVOset = updateConnection(SVO.vVOset, viewId-1, viewId, ...
              'Matches', idxMatch,  'Orientation', rmat', 'Location', tvec);
          
%% =======================================================================
%  Bundle Adjustment ....
%  =======================================================================
if params.BA.activate
    fprintf('Processing Bundle Adjustment ... \n')
    OrientationBA = SVO.vVOset.Views.Orientation{end} * R;
    LocationBA = SVO.vVOset.Views.Location{end} + (OrientationBA * T')';      
    
    SVO.vVOsetBA = updateView(SVO.vVOsetBA, viewId, 'Points', cat(1, currState.keyPoints.keyPointsLeft.pt), ... 
            'Orientation', OrientationBA, 'Location', LocationBA);
        
    % Add the conection between both intstans of time
    SVO.vVOsetBA = updateConnection(SVO.vVOsetBA, viewId-1, viewId, ...
              'Matches', idxMatch, 'Orientation', rmat', 'Location', tvec);
    
    % WINDOWED BOUNDLE ADJUSTMENT
    if mod(viewId, params.BA.window) == 0 % Windowed BA every 10 views
        % Find point tracks in the last 15 views and triangulate.
        startFrame = max(1, viewId - params.BA.window);
        tracks = findTracks(SVO.vVOset, startFrame:viewId);
        camPoses = poses(SVO.vVOset, startFrame:viewId);
        [xyzMultiview, reprojError] = triangulateMultiview(tracks, ...
                                                           camPoses, ...
                                                           dataset.PL_params);

        % Hold the first two poses fixed, to keep the same scale. 
        fixedIds = startFrame : startFrame+1;
        if fixedIds(1) <= 0 
            fixedIds = startFrame : startFrame+1;
        end

        % Exclude points and tracks with high reprojection errors.
        idxLowError = reprojError < 2;

        % Solve Bundle Adjustment
        [~, camPoses, ~] = bundleAdjustment(xyzMultiview(idxLowError, :), ...
                           tracks(idxLowError), camPoses, dataset.PL_params, ...
                           'FixedViewIDs', fixedIds, ...
                           'PointsUndistorted', params.BA.PointsUndistorted, ...
                           'AbsoluteTolerance', params.BA.AbsoluteTolerance, ...
                           'RelativeTolerance', params.BA.RelativeTolerance, ...
                           'MaxIterations', params.BA.MaxIterations);

        % Update Camera Poses 
        SVO.vVOsetBA = updateView(SVO.vVOsetBA, camPoses); % Update view set.  
        
    end
    
    % Calculate the error compared to Ground Truth
    loc_GT = dataset.vGTset.Views.Location{viewId};
    loc_BA = SVO.vVOsetBA.Views.Location{viewId};%    .* [1 -1 1];
    SVO.errorBA.X(viewId,1) = abs(loc_GT(1) - loc_BA(1));
    SVO.errorBA.Y(viewId,1) = abs(loc_GT(2) - loc_BA(2));
    SVO.errorBA.Z(viewId,1) = abs(loc_GT(3) - loc_BA(3));
    
end
    
%% =======================================================================
%  Based on the P3P solver, filter the input data and prepare for the next
%  iteration....................
%  =======================================================================
% Current landmarks information:
currState.keypoints = currState.tracked_P3P_Left(idxPose,:);
currState.landmarks = prevState.xyzP3P(idxPose,:);

% Calculate the error compared to Ground Truth
loc_GT = dataset.vGTset.Views.Location{viewId};
loc_PE = SVO.vVOset.Views.Location{viewId};%    .* [1 -1 1];
SVO.error.X(viewId,1) = abs(loc_GT(1) - loc_PE(1));
SVO.error.Y(viewId,1) = abs(loc_GT(2) - loc_PE(2));
SVO.error.Z(viewId,1) = abs(loc_GT(3) - loc_PE(3));

%% Print Status .....
fprintf('Frame processing done .... \n')
fprintf('Ground Truth location:     X = %4.3f, Y = %4.3f, Z = %4.3f \n', ...
        loc_GT(1), loc_GT(2), loc_GT(3))
fprintf('Current location:          X = %4.3f, Y = %4.3f, Z = %4.3f \n', ...
        currState.Location(1), currState.Location(2), currState.Location(3))
    
if params.BA.activate
    locBA = SVO.vVOsetBA.Views.Location{end};
    fprintf('Current BA loc:            X = %4.3f, Y = %4.3f, Z = %4.3f \n', ...
        locBA(1), locBA(2), locBA(3));
end

%% Calculate Displacement and Velocity
Displacement = SVO.Displacement(1:end-1,:);
SVO.Displacement = [Displacement; norm(T_oCV)];
deltaTime = 1 / params.data.Frequency;
Velocity = SVO.Velocity(1:end-1,:);
SVO.Velocity = [Velocity ; SVO.Displacement(end) / deltaTime];

fprintf('============================================================= \n')

if params.debugging.plotting
    close(100); close(101); close(102); close(103)
end
end
% End of Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~