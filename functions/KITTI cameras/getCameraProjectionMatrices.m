%% getCameraProjectionMatrices ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This function Creates projection matrices for a stereo
    camera. It is assumed that the parameters are for rectified images.

    * I/O       * Objects       * Description   
    Inputs:     - cam_params    - Camera parameters structure comprising of 
                                  fx, fy, cx, cy, baseline  

    Outputs:    - PL            - Camera Projecttion Matrix (3x4) Left
                - PR            - Camera Projecttion Matrix (3x4) Right
                
    Helper Function: N/A

    Created by: Fabian Aguilar.
    Date:       02/24/19
    Edition:    3
    Edit Date:  08/03/20
%}

%% Start Funtion ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [PL, PR] = getCameraProjectionMatrices(cam_params)

% Left camera
PL = [cam_params.fx,    0,              cam_params.cx,  0; ...
      0,                cam_params.fy,  cam_params.cy,  0; ...
      0,                0,              1,              0];

% Right camera
PR = [cam_params.fx,    0,              cam_params.cx,  -cam_params.base; ...
      0,                cam_params.fy,  cam_params.cy,  0; ...
      0,                0,              1,              0];
end
% End Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
