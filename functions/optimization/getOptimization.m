%% getOptimization ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    This function performs a Non-Linear Optimization using a Non-Linear 
    Least Squares aproach. The algorithm uses the Levenberg-Marquardt
    Method implemented on the MATLAB Optimization toolbox. 

    * I/O       * Objects       * Description   
    Inputs:     - R             - Rotation matrix [3 x 3].
                - T             - Translation vector [1 x 3].
                - idx           - Index corresponding to the output of 
                                  valid points from P3P solver.
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

    Outputs:    - R_opt,        - Optimized Rotation Matrix             
                - Tr_opt        - Optimized Translation Vector

    Subfuntions:
    - lsqnonlin (MATLAB)
    - rotm2quat (MATLAB)
    - getMinimization
    - optimoptions (MATLAB)

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
function [R_opt, Tr_opt] = getOptimization(R, T, idx, landmarks, ...
                                keypointsLeft, PL, keypointsRight, PR)

% Transform rotation matrix into quaterion
quatirion = rotm2quat(R);

% Make a translation rotation vector
t_r_v = [T'; quatirion'];

% Use non-linear optimization (least squares) to minimize reprojection error
% argmin?????? ?(????,??????,??1??2 + ?????,??????,??1??2) EQ 6.3 Thesis

% Initialise for lsqnonlin
f = @(T_R_V)getMinimization(T_R_V, double(landmarks(idx,:)), ...
                              double(keypointsLeft(idx,:)), PL, ...
                              double(keypointsRight(idx,:)), PR);

% Set options for lsqnonlin 
% TODO: Try more optimzation models
options = optimoptions(@lsqnonlin, 'Algorithm','levenberg-marquardt',... 'ScaleProblem', 'jacobian', ...
          'Display','off','FunValCheck','on','MaxIter',400, ...
          'FiniteDifferenceType', 'central', 'UseParallel', false);
      
% Run Leasts-Squared solver 
[TR, squarederrornorm, errors, exitflag, iter] = lsqnonlin(f, t_r_v, [], [], options);

% Unwrap optimized Translation and Rotation vector
Tr_opt = TR(1:3,1)';

% Transform from quaterion to matrix
R_opt = quat2rotm(TR(4:end,1)');

fprintf('Iterations taken by LSQ Opt: %4d \n', iter.iterations);

end
% End of the Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 