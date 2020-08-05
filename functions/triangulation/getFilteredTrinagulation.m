%% getFilteredTrinagulation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This function filter the triangulated values such that only values that
    lie within a max radius from the camera pose will be consider. All
    values must lie infrot of the camera.

    * I/O       * Objects       * Description   
    Inputs:     - xyzPoints     - 3D World camera coordinates from DLT
                - reproError    - Reprojection error from triangulation.
                - maxRadius     - Max radius from the camera pose.
                - minDistThresh - Min distance threshold.
                - repErrThresh  - Reprojection error Threshold

    Outputs:    - xyzFilter     - 3D World camera coordinates filtered.
                - idxFilter     - Index of valied filtered values.

    Created by: Fabian Aguilar.
    Date:       02/24/19
    Edition:    3
    Edit Date:  08/03/20
%}

%% Start Funtion ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [xyzFilter, idxFilter, ratioFilter] = getFilteredTrinagulation(...
            xyzPoints, reproError, maxRadius, minDistThresh, repErrThresh)

% Set conditions:
condition1 = sqrt(sum(xyzPoints .^ 2, 2)) <= maxRadius;
condition2 = xyzPoints(:,3) > minDistThresh;
condition3 = xyzPoints(:,1) > -12 & xyzPoints(:,1) < 12; % Before [-12 12 ]
condition4 = xyzPoints(:,2) < 2 & xyzPoints(:,2) > -8; % Before [2 -6]
condition5 = reproError < repErrThresh;

% Conditional to be a considered value
conditions = and(condition1,and(condition2, and(condition3, and(condition4, condition5))));
% conditions = and(condition1,and(condition2, condition5));

% Get indices of valid/invalid points
idxValid = find(conditions); 
% idxInvalid = find(~conditions);

% Get ratio of useful 3D points
ratioFilter = length(idxValid)/length(condition1);

% Index Output
idxFilter = idxValid;

% 3D World camera coordinates filtered
xyzFilter = xyzPoints(idxFilter,:);  

end
% End of the Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~