%% getMinimization ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This function performs the minimization of the reprojection error from
    the landmarks into the left and the right images

    * I/O       * Objects       * Description   
    Inputs:     - t_r_v         - Translation & Rotation vector [7 x 1]
                - landmarks     - Input 3D points corresponding to the 
                                  triangulated correspondaces of keypointsLeft, 
                                  and keypointsRight in the form [Mx3].
                                  This must be the same input as ones used 
                                  for P3P solver.
                - keypointsLeft - Input 2D points tracked at current instant 
                                  of time over the left image, in the form 
                                  [Mx2]. This must be the same input as the 
                                  ones used for P3P solver.
                - PL            - Projection matrix for Left Camera
                - keypointsRight - Input 2D points tracked at current instant 
                                  of time over the right image, in the form 
                                  [Mx2]. This must be the same input as the 
                                  ones used for P3P solver.
                - PR            - Projection matrix for Right Camera

    Outputs:    - reprojErro    - Difference between keypoints and projected
                                  landmarks into camera coordinates frame.

    Subfuntions:
    - hypot (MATLAB)
    - projectPoints

    NOTE: FOR FURTHER READING, PLEASE REFER TO CHAPTER 6 IN [1]

    [1] E. F. Aguilar Calzadillas, "Sparse Stereo Visual Odometry 
        with Local Non-Linear Least-Squares Optimization for 
        Navigation of Autonomous Vehicles", M.A.Sc. Thesis, 
        Depart. of Mech. and Aero. Eng., Carleton University, 
        Ottawa ON, Canada, 2019. [Online] Available: 
        https://curve.carleton.ca/7270ba62-1fd3-4f1b-a1fa-6031b06585e9

    Created by: Fabian Aguilar.
    Date:       03/19/19
    Edition:    3
    Edit Date:  08/03/20
%}

%% Start Funtion ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [reprojError] = getMinimization(t_r_v, landmarks, ...
                                     keypointsLeft, PL, keypointsRight, PR)

% Unwrap the translation rotation vector, which is in World coordinates
R_W_C = quat2rotm(t_r_v(4:end,1)');     % Rotation Matrix in World Coordinates
t_W_C = t_r_v(1:3,1);                   % Translation vector in World Coordinates

% Now, convert the Rotation and Translation from World Coordinates into 
% Camera Coordinates
R_C_C = R_W_C';                         % Rotation Matrix in Camera Coordinates
t_C_C = -R_C_C * t_W_C;                 % Translation vector in Camera Coordinates

%% Project landmarks to camera coordinates frame
landmarks_C_C = (R_C_C * landmarks') + repmat(t_C_C, [1 size(landmarks, 1)]);

% Convert to pixel coordinates using the corresponding Projection Matrix
points1proj = projectPoints(landmarks_C_C', PL');
points2proj = projectPoints(landmarks_C_C', PR');

%% Sum of Squared differences >> EQ 6.3
keypointsLeft = keypointsLeft';
keypointsRight = keypointsRight';

errors1 = hypot(keypointsLeft(1,:)- points1proj(1,:), ...
                keypointsLeft(2,:) - points1proj(2,:));
errors2 = hypot(keypointsRight(1,:)- points2proj(1,:), ...
                keypointsRight(2,:) - points2proj(2,:));

reprojError = mean([errors1; errors2])';
end
% End of the Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 


%% Helper Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function points2d = projectPoints(points3d, P)
points3dHomog = [points3d, ones(size(points3d, 1), 1, 'like', points3d)]';
points2dHomog = P * points3dHomog;
points2d = bsxfun(@rdivide, points2dHomog(1:2, :), points2dHomog(3, :));
end