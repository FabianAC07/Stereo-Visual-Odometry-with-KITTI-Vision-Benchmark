%% getDLT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This functions performs the algorithm Direct Linear Transformation
    (DLT) implemented in the OpenCV fucntion cv.triangulatePoints()

    * I/O       * Objects       * Description   
    Inputs:     - state         - Struct object with the information of the 
                                  given instant of time
                - dataset       - Struct containing the stereo parameters
                                  of the stereo camera

    Outputs:    - worldPoints   - Triangulated points or 3D keypoints
          - reprojectionErrors  - Reprojection error given by the
                                  reprojection of worldPoints into 
                                  image plane
                
    Helper Function: projectPoints

    Created by: Fabian Aguilar.
    Date:       02/24/19
    Edition:    3
    Edit Date:  08/03/20
%}

%% Start function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

function [worldPoints, reprojectionErrors] = getDLT(state, dataset)

% Retrive inliers at left and right images
p1 = state.Inliers.Left;
p2 = state.Inliers.Right;

% Perform DLT 
points4D = cv.triangulatePoints(dataset.PL, dataset.PR, ...
                    num2cell(p1,2), num2cell(p2,2));

% Normalize the homogeneous points
Points3D = points4D ./ points4D(4,:);
Points3D = Points3D(1:3,:);

% Compute reprojection errors
points1proj = projectPoints(Points3D', dataset.PL);
points2proj = projectPoints(Points3D', dataset.PR);

points1 = state.Inliers.Left';
points2 = state.Inliers.Right';

errors1 = hypot(points1(1,:) - points1proj(1,:), ...
                points1(2,:) - points1proj(2,:));
errors2 = hypot(points2(1,:) - points2proj(1,:), ...
                points2(2,:) - points2proj(2,:));

reprojectionErrors = mean([errors1; errors2])';

% World coordinates of the inliers.
worldPoints = Points3D';

end
% End Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

%% Helper Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function points2d = projectPoints(points3d, P)
points3dHomog = [points3d, ones(size(points3d, 1), 1, 'like', points3d)]';
points2dHomog = P * points3dHomog;
points2d = bsxfun(@rdivide, points2dHomog(1:2, :), points2dHomog(3, :));
end