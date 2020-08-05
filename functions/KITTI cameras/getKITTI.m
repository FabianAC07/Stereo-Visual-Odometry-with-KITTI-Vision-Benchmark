%% getKITTI ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This functions loads the KITTI Dataset and its camera parameters:

    * I/O       * Objects       * Description   
    Inputs:     - caller        - ZED or ZED Mini 

    Outputs:    - dataset       - Struct containing the stereo parameters
                                  of the stereo camera
                
    Helper Function: N/A

    Created by: Fabian Aguilar.
    Date:       02/24/19
    Edition:    3
    Edit Date:  08/03/20
%}

%% Start function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [dataset] = getKITTI(data)

% Add path to KITTI dataset
addpath(genpath('dataset'))

% Load images depending on the DSS value
switch(data.Sequence)
    case 0 % Sequence 00
        data_params.path1 = 'dataset/sequences/00/image_0/';
        data_params.path2 = 'dataset/sequences/00/image_1/';
        if data.GroundTruth % Load GT if flag is 1.
            data_params.gt_file = 'dataset/poses/00.txt';
            ground_truth = load(data_params.gt_file);
        end
    case 1 % Sequence 01
        data_params.path1 = 'dataset/sequences/01/image_0/';
        data_params.path2 = 'dataset/sequences/01/image_1/';
        if data.GroundTruth % Load GT if flag is 1.
            data_params.gt_file = 'dataset/poses/01.txt';
            ground_truth = load(data_params.gt_file);
        end
    case 2 % Sequence 02
        data_params.path1 = 'dataset/sequences/02/image_0/';
        data_params.path2 = 'dataset/sequences/02/image_1/';
        if data.GroundTruth % Load GT if flag is 1.
            data_params.gt_file = 'dataset/poses/02.txt';
            ground_truth = load(data_params.gt_file);
        end
    case 3 % Sequence 03
        data_params.path1 = 'dataset/sequences/03/image_0/';
        data_params.path2 = 'dataset/sequences/03/image_1/';
        if data.GroundTruth % Load GT if flag is 1.
            data_params.gt_file = 'dataset/poses/03.txt';
            ground_truth = load(data_params.gt_file);
        end
    case 4 % Sequence 04
        data_params.path1 = 'dataset/sequences/04/image_0/';
        data_params.path2 = 'dataset/sequences/04/image_1/';
        if data.GroundTruth % Load GT if flag is 1.
            data_params.gt_file = 'dataset/poses/04.txt';
            ground_truth = load(data_params.gt_file);
        end
    case 5 % Sequence 05
        data_params.path1 = 'dataset/sequences/05/image_0/';
        data_params.path2 = 'dataset/sequences/05/image_1/';
        if data.GroundTruth % Load GT if flag is 1.
            data_params.gt_file = 'dataset/poses/05.txt';
            ground_truth = load(data_params.gt_file);
        end
    case 6 % Sequence 06
        data_params.path1 = 'dataset/sequences/06/image_0/';
        data_params.path2 = 'dataset/sequences/06/image_1/';
        if data.GroundTruth % Load GT if flag is 1.
            data_params.gt_file = 'dataset/poses/06.txt';
            ground_truth = load(data_params.gt_file);
        end
    case 7 % Sequence 07
        data_params.path1 = 'dataset/sequences/07/image_0/';
        data_params.path2 = 'dataset/sequences/07/image_1/';
        if data.GroundTruth % Load GT if flag is 1.
            data_params.gt_file = 'dataset/poses/07.txt';
            ground_truth = load(data_params.gt_file);
        end
    case 8 % Sequence 08
        data_params.path1 = 'dataset/sequences/08/image_0/';
        data_params.path2 = 'dataset/sequences/08/image_1/';
        if data.GroundTruth % Load GT if flag is 1.
            data_params.gt_file = 'dataset/poses/08.txt';
            ground_truth = load(data_params.gt_file);
        end
    case 9 % Sequence 09
        data_params.path1 = 'dataset/sequences/09/image_0/';
        data_params.path2 = 'dataset/sequences/09/image_1/';
        if data.GroundTruth % Load GT if flag is 1.
            data_params.gt_file = 'dataset/poses/09.txt';
            ground_truth = load(data_params.gt_file);
        end
    case 10 % Sequence 10
        data_params.path1 = 'dataset/sequences/10/image_0/';
        data_params.path2 = 'dataset/sequences/10/image_1/';
        if data.GroundTruth % Load GT if flag is 1.
            data_params.gt_file = 'dataset/poses/10.txt';
            ground_truth = load(data_params.gt_file);
        end
        
    % TODO: Implement the code for the sequences with no Ground Truth -
    % Sequences from 11 to 21
end

% Sotre Ground Truth for processing comparisson
vGTset = viewSet;

% Retrive the ground truth poses from file
for i = 1 : length(ground_truth) - 1
    GT = reshape(ground_truth(i, :), 4, 3)';
    Orientation_GT = GT(:,1:3);
    Location_GT = GT(:, 4);
        
    % Add GT data to a vGTset 
    vGTset = addView(vGTset, i, 'Orientation', Orientation_GT,...
                'Location', Location_GT');
end

% Store the dataset information in an struct object 
dataset.vGTset = vGTset;

% Calibration parameters for sequence 2010_03_09_drive_0000
cam_params.fx = 7.188560000000e+02;   % focal length (u-coordinate) in pixels
cam_params.cx = 6.071928000000e+02;   % principal point (u-coordinate) in pixels
cam_params.fy = 7.188560000000e+02;   % focal length (v-coordinate) in pixels
cam_params.cy = 1.852157000000e+02;   % principal point (v-coordinate) in pixels
cam_params.base = 3.861448000000e+02; % baseline in meters (absolute value)

% Read directories containing images
dataset.img_dir_L = dir(strcat(data_params.path1,'*.png'));
dataset.img_dir_R = dir(strcat(data_params.path2,'*.png'));

% Bould Projection Camera Matrices
[dataset.PL, dataset.PR] = getCameraProjectionMatrices(cam_params);

% Build Camera Matrices
img = imread([dataset.img_dir_L(1).folder, '/', dataset.img_dir_L(1).name]);
[row, col] = size(img);

% For KITTI the camera left has the same intrinsics than right camera 
dataset.PL_intrinsics = cameraIntrinsics([cam_params.fx, cam_params.fy], ...
                [cam_params.cx, cam_params.cy], [row, col]);
dataset.PR_intrinsics = dataset.PL_intrinsics;

% Build Camera parameters accordingly to matlab
dataset.PL_params = cameraParameters('IntrinsicMatrix', ...
                        dataset.PL_intrinsics.IntrinsicMatrix);
dataset.PL_params.ImageSize = [row, col];
dataset.PR_params = cameraParameters('IntrinsicMatrix', ...
                        dataset.PR_intrinsics.IntrinsicMatrix);
dataset.PR_params.ImageSize = [row, col];

% Build Stereo Camera parameters accordingly to matlab
dataset.stereoParams = stereoParameters(dataset.PL_params, ...
                       dataset.PR_params,eye(3), [-cam_params.base 0 0]);

end
% End Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~