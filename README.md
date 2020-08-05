# Stereo Visual Odometry with KITTI Vision Benchmark

This repository is a MATLAB implementation as part of the Master Thesis Project [Sparse Stereo Visual Odometry with Local Non-Linear Least-Squares Optimization for Navigation of Autonomous Vehicles](https://curve.carleton.ca/7270ba62-1fd3-4f1b-a1fa-6031b06585e9). 

The code has been set up to be used with the [KITTI Vision Benchmark Suit](http://www.cvlibs.net/datasets/kitti/).

The code has been written tested on [MATLAB R2019a](https://www.mathworks.com/?s_tid=gn_logo) and depends on the following toolboxes:
* Computer Vision Toolbox
* Image Processing Toolbox
* Optimization Toolbox

Also, the code uses [OpenCV 3.4.1](https://opencv.org/opencv-3-4-1/) libraries which are implementen in [MEX files](https://www.mathworks.com/help/matlab/call-mex-file-functions.html#:~:text=A%20MEX%20file%20is%20a,only%20one%20function%20or%20subroutine.).
 
The software was tested on a laptop `Intel(R) Core(TM) i7-6500U CPU @ 2.50GHz 2.60 GHz` and `12 GB RAM`.

## How to run the repository?

1. Clone the repository using the following command:
```
git clone https://github.com/FabianAC07/Stereo-Visual-Odometry-with-KITTI-Vision-Benchmark
```

2. Install [mexopencv](https://github.com/kyamagu/mexopencv)

3. Import the dataset to the folder [`data`](data). You can request a download for KITTI "data_odometry_gray.zip" dataset [here](http://www.cvlibs.net/download.php?file=data_odometry_gray.zip)

4. Change the corresponding paramters in the parameters [`parameters.m`](src/parameters.m) file according to your needs

5. Run the script [`main.m`](src/main.m)

6. Depending on your choice in the parametes file, you might get a plot of the Visual Odometry estimation during sequence processing

7. Once the process is done, a new file `scr/results` will be created. It will host all the outputs that you request in the parameters file, which can include: 

* Video of the sequence
* Command Window Output
* Workspace Output for further post-processing

8. Use the script [plot_results.m](src/plot_results.m) to plot the results from Workspace Output file.

### Video Demo

The following video is a demo of the plot output while processing...

https://www.youtube.com/watch?v=NaQ8bB9eXo4


## Further Reading

For details on the implemenation and use of this software please refer to the M.A.Sc. Thesis [Sparse Stereo Visual Odometry with Local Non-Linear Least-Squares Optimization for Navigation of Autonomous Vehicles](https://curve.carleton.ca/7270ba62-1fd3-4f1b-a1fa-6031b06585e9). 


## License

This software is under GNU General Public License v3.0 License.

If you use this software in an academic work, please cite:

E. F. Aguilar Calzadillas, "Sparse Stereo Visual Odometry with Local Non-Linear Least-Squares Optimization for Navigation of Autonomous Vehicles", M.A.Sc. Thesis, Depart. of Mech. and Aero. Eng., Carleton University, Ottawa ON, Canada, 2019.
