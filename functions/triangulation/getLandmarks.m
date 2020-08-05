%%  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This function transforms the landmarks at the current instant of time
    into landmarks with respect to the previous image frame, to add them to
    the global pointcloud.

    * I/O       * Objects       * Description   
    Inputs:     - SVO           - SVO object to store Location and Mapping 
                - landmarks     - Landmarks calculated at a current instat
                                  of time

    Outputs:    - SVO           - SVO object to store Location and Mapping 
                - i             - Number of landmamarks

    Created by: Fabian Aguilar.
    Date:       02/25/19
    Edition:    3
    Edit Date:  08/03/20
%}

%% Start Funtion ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [SVO, i] = getLandmarks(SVO, landmarks)

% Retrive last pose
R = SVO.vVOset.Views.Orientation{end};
T = SVO.vVOset.Views.Location{end};

% Translate landmarks wrp to the previous image frame
for i = 1 : length(landmarks(:,1))
    landmark = T + (R * landmarks(i,:)')';
    SVO.landmarks = [SVO.landmarks ; landmark];
end

SVO.poseLandmarks = [SVO.poseLandmarks ; repmat([R(:)', ...
    T(:)'], [length(landmarks),1])];

% Store total number of landmarks
SVO.totLandmarks = length(SVO.landmarks);

end 
% End of Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~