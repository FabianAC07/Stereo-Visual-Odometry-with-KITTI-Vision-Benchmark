%%  getClose ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This function check if the tracked keypoints at a current instant of 
    are close to the keypoints stored converted to landmarks from previous 
    instants of time.

    * I/O       * Objects       * Description   
    Inputs:     - query         - Array of 2D to test if it is close to
                                  the reference input.
                - reference     - Array of 2D points (Reference)
                - treshold      - Max distance between query and reference 
                                  inputs  

    Outputs:    - idxClose      - Logical array of indices close to the
                                  reference

    Created by: Fabian Aguilar.
    Date:       03/09/19
    Edition:    3
    Edit Date:  08/03/20
%}

%% Start Funtion ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function idxClose = getClose(query, reference, treshold)

query_1 = size(query , 1);
reference_1 = size(reference, 1);

idxClose = zeros(query_1, 1);

for i= 1 : query_1
    for j= 1 : reference_1
        if ((query(i, 1) - reference(j, 1))^2 + (query(i, 2) - reference(j, 2))^2) < treshold^2
            idxClose(i) = true; 
            break; 
        end
    end
end


end
% End of function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~