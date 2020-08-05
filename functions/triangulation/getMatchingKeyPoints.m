%% getMatchingKeyPoints ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This function extract points and its feature descriptors form the
    stereo images

    * I/O       * Objects       * Description   
    Inputs:     - frames        - Object containing the stereo frames 
                - parameters    - Object containing process parameters
                - debugging     - Object containing debugging parameters

    Outputs:    - matchedPoints - Object containing valid matching points
                                  in the left and right images
                - keyPoints     - Object containing all the 2D image points
                                  found in the left and right images
                - descriptor    - Object containing the descriptor of the
                                  keyPoints object

    Subfuntions:
    - detectHarrisFeatures (MATLAB).
    - detectSURFFeatures (MATLAB).
    - detectBRISKFeatures (MATLAB).
    - selectUniform (MATLAB).
    - getNonMaxSuppression.
    - cornerPoints (MATLAB).
    - extractFeatures (MATLAB).
    - matchFeatures (MATLAB).

    Created by: Fabian Aguilar.
    Date:       02/24/19
    Edition:    3
    Edit Date:  08/03/20
%}

%% Start Funtion ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [matchedPoints, keyPoints, descriptor] = ...
            getMatchingKeyPoints(frames, params)       

% Check if the frame is RGB, if so, convert to gray scale:
[~, ~, n] = size(frames.Left);
if n == 3
    % Convert images to gray scale 
    frames.Left = cv.cvtColor(frames.Left, 'RGB2GRAY');
    frames.Right = cv.cvtColor(frames.Right, 'RGB2GRAY');
end

%% Initialize Feature Detector
switch(params.opencv.features)
    %% Good Features to Track
    case 'HARRIS'
        featuresLeft = cv.goodFeaturesToTrack(frames.Left, ...
                        'MaxCorners', params.opencv.GFTT.MaxCorners, ...
                        'QualityLevel', params.opencv.GFTT.QualityLevel, ...
                        'MinDistance', params.opencv.GFTT.MinDistance, ...
                        'BlockSize', params.opencv.GFTT.BlockSize, ...
                        'UseHarrisDetector', params.opencv.GFTT.UseHarrisDetector);
                    
        featuresLeft = cv.cornerSubPix(frames.Left, featuresLeft);

        % Tracking Left to Right
        p0 = cellfun(@(tr) tr(end,:), featuresLeft, 'UniformOutput',false);
        p1 = cv.calcOpticalFlowPyrLK(frames.Left, frames.Right, p0, params.opencv.lk_params{:});
        p0r = cv.calcOpticalFlowPyrLK(frames.Right, frames.Left, p1, params.opencv.lk_params{:});
        
        % keep only good matches
        idxGoodMatch = cellfun(@(a,b) max(abs(a - b)), p0, p0r) < 1;
        
        featuresLeft = featuresLeft(idxGoodMatch);
        featuresRight = p1(idxGoodMatch);
        
        keyPointsLeft = cv.KeyPointsFilter.convertFromPoints(featuresLeft);
        keyPointsRight = cv.KeyPointsFilter.convertFromPoints(featuresRight);

        if params.debugging.checkFeatures
            figure
            subplot(121), imshow(cv.drawKeypoints(frames.Left, keyPointsLeft))
            subplot(122), imshow(cv.drawKeypoints(frames.Right, keyPointsRight))
        end

        % Get descriptors
        extractor = cv.DescriptorExtractor(params.opencv.GFTT.BriefDescriptorExtractor, ...
                        'Bytes', params.opencv.GFTT.Bytes, ...
                        'UseOrientation', params.opencv.GFTT.UseOrientation);
        
        descriptorsLeft = extractor.compute(frames.Left, keyPointsLeft);
        descriptorsRight = extractor.compute(frames.Right, keyPointsRight);
        
        % Match Features based on their descriptors: 
        
%         matcher = cv.DescriptorMatcher('FlannBasedMatcher', ...
%                          'Index',{'KDTree', 'Trees',5}, 'Search',{'Checks',50});
% %         matcher = cv.DescriptorMatcher('BruteForce');
%         matches = matcher.knnMatch(descriptorsLeft, descriptorsRight, 2);
% 
%         idx = cellfun(@(m) m(1).distance < 0.75 * m(2).distance, matches);
%         matches = cellfun(@(m) m(1), matches(idx));
        
        % Output
        descriptor.descriptorsLeft = descriptorsLeft;
        descriptor.descriptorsRight = descriptorsRight;
        keyPoints.keyPointsLeft = keyPointsLeft;
        keyPoints.keyPointsRight = keyPointsRight;        
        matchedPoints.Left = cat(1, featuresLeft{:});                
        matchedPoints.Right = cat(1, featuresRight{:});

    %% SURF 
    case 'SURF'
        % Create SURF detector                    
        detector = cv.FeatureDetector(params.opencv.features, ...
                    'Extended', params.opencv.SURF.Extended, ...
                    'HessianThreshold', params.opencv.SURF.HessianThreshold, ...
                    'NOctaves', params.opencv.SURF.NOctaves, ...
                    'NOctaveLayers', params.opencv.SURF.NOctaveLayers, ...
                    'Upright', params.opencv.SURF.Upright);
                    
        keyPointsLeft = detector.detect(frames.Left);
        keyPointsRight = detector.detect(frames.Right);
        
        if params.debugging.checkFeatures
            %HACK: HISTOGRAM not implemented in Octave
            figure(110)
            histogram([keyPointsLeft.size]), hold on
            histogram([keyPointsRight.size])
            xlabel('Keypoint sizes'), ylabel('Count')
            legend('keypoints1', 'keypoints2')
            hold off
        end
        
        % Filter key points by Size
        keyPointsLeft = cv.KeyPointsFilter.runByKeypointSize(keyPointsLeft, ...
                                    0, params.opencv.SURF.runByKeypointSize);
        keyPointsRight = cv.KeyPointsFilter.runByKeypointSize(keyPointsRight, ...
                                    0, params.opencv.SURF.runByKeypointSize);
        
        if params.debugging.checkFeatures
            %HACK: HISTOGRAM not implemented in Octave
            histogram([keyPointsLeft.response]), hold on
            histogram([keyPointsRight.response])
            xlabel('Keypoint responses'), ylabel('Count')
            legend('keypoints1', 'keypoints2')
            hold off
        end
        
        % Filter keypoints by responses
        keyPointsLeft = cv.KeyPointsFilter.retainBest(keyPointsLeft, params.opencv.SURF.sample);
        keyPointsRight = cv.KeyPointsFilter.retainBest(keyPointsRight, params.opencv.SURF.sample);
        
        % Remove duplucated points
        keyPointsLeft = cv.KeyPointsFilter.removeDuplicatedSorted(keyPointsLeft);
        keyPointsRight = cv.KeyPointsFilter.removeDuplicatedSorted(keyPointsRight);
        
        if params.debugging.checkFeatures
            figure(111)
            subplot(121), imshow(cv.drawKeypoints(frames.Left, keyPointsLeft))
            subplot(122), imshow(cv.drawKeypoints(frames.Right, keyPointsRight))
        end
        
        % Get descriptors
        extractor = cv.DescriptorExtractor(params.opencv.features, ...
                    'Extended', params.opencv.SURF.Extended, ...
                    'HessianThreshold', params.opencv.SURF.HessianThreshold, ...
                    'NOctaves', params.opencv.SURF.NOctaves, ...
                    'NOctaveLayers', params.opencv.SURF.NOctaveLayers, ...
                    'Upright', params.opencv.SURF.Upright);

        descriptorsLeft = extractor.compute(frames.Left, keyPointsLeft);
        descriptorsRight = extractor.compute(frames.Right, keyPointsRight);
        
        % Match Features based on their descriptors:      
        matcher = cv.DescriptorMatcher('FlannBasedMatcher', ...
                         'Index',{'KDTree', 'Trees',2}, 'Search',{'Checks', 50});
%         matcher = cv.DescriptorMatcher('BruteForce');
        matches = matcher.knnMatch(descriptorsLeft, descriptorsRight, 2);

        idx = cellfun(@(m) m(1).distance < params.opencv.SURF.MaxRatio * m(2).distance, matches);
        matches = cellfun(@(m) m(1), matches(idx));
        
        if params.debugging.checkFeatures
            figure(112)
            histogram([matches.distance])
            xlabel('Match distances'), ylabel('Count')
        end
        
        % Filter matches by distance ("good" matches)
        if true
            [~,idx] = sort([matches.distance]);
%             idx = idx(1:min(1000,end));
            matches = matches(idx);
        else
            min_dist = min([matches.distance]);
            matches = matches([matches.distance] <= max(3*min_dist, 0.22));
        end
        
        if params.debugging.checkFeatures
            % draw matches and show result
            out = cv.drawMatches(frames.Left, keyPointsLeft, frames.Right, keyPointsRight, matches);
            figure(113), imshow(out)
        end
        
        
        % Output
        matchedPoints.Left = cat(1, keyPointsLeft([matches.queryIdx]+1).pt);
        matchedPoints.Right = cat(1, keyPointsRight([matches.trainIdx]+1).pt);
        descriptor.descriptorsLeft = descriptorsLeft;
        descriptor.descriptorsRight = descriptorsRight;
        keyPoints.keyPointsLeft = keyPointsLeft;
        keyPoints.keyPointsRight = keyPointsRight;
        
        if params.debugging.checkFeatures
            close(110); close(111); close(112); close(113)
        end

    %% ORB
    case 'ORB'
        % ORB object
        ORB = cv.ORB('MaxFeatures', params.opencv.ORB.MaxFeatures, ...
                     'ScaleFactor', params.opencv.ORB.ScaleFactor, ...
                     'NLevels', params.opencv.ORB.NLevels, ...
                     'EdgeThreshold', params.opencv.ORB.EdgeThreshold, ...
                     'FirstLevel', params.opencv.ORB.FirstLevel, ...
                     'WTA_K', params.opencv.ORB.WTA_K, ...
                     'ScoreType', params.opencv.ORB.ScoreType, ...
                     'PatchSize', params.opencv.ORB.PatchSize, ...
                     'FastThreshold', params.opencv.ORB.FastThreshold);
        
        % detect and compute descriptors in both images
        [keyPointsLeft, descriptorsLeft] = ORB.detectAndCompute(frames.Left);
        [keyPointsRight, descriptorsRight] = ORB.detectAndCompute(frames.Right);
        
        % Match Features based on their descriptors:        
        matcher = cv.DescriptorMatcher('BruteForce-Hamming');
        matches = matcher.match(descriptorsLeft, descriptorsRight);
        
        if params.debugging.checkFeatures
            figure
            histogram([matches.distance])
            xlabel('Match distances'), ylabel('Count')
        end
        
        % Filter matches by distance ("good" matches)
        if true
            [~,idx] = sort([matches.distance]);
%             idx = idx(1:min(1000,end));
            matches = matches(idx);
        else
            min_dist = min([matches.distance]);
            matches = matches([matches.distance] <= max(3*min_dist, 0.22));
        end
        
        if params.debugging.checkFeatures
            % draw matches and show result
            out = cv.drawMatches(frames.Left, keyPointsLeft, frames.Right, keyPointsRight, matches);
            figure, imshow(out)
        end
                
        % Output
        matchedPoints.Left = cat(1, keyPointsLeft([matches.queryIdx]+1).pt);
        matchedPoints.Right = cat(1, keyPointsRight([matches.trainIdx]+1).pt);
        descriptor.descriptorsLeft = descriptorsLeft;
        descriptor.descriptorsRight = descriptorsRight;
        keyPoints.keyPointsLeft = keyPointsLeft;
        keyPoints.keyPointsRight = keyPointsRight;       
end

end
% End Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        