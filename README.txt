# Save rectified images

This package is dedicated to save rectified images from ROSbag

Tested in ROS Kinetic + OpenCV 3.2 + C++ 11

# Installation
Clone in your catkin envirronment and compile! 

# Usage

This package can be used for different purposes:
1. Stereo rectification
2. Double stereo rectification:
3. monocular rectification:
`rosrun rectif_save rectif_mono _Path_Intrinsic:="PATH_TO_INTRINSICS"`

For monocular rectification, make sure to create a file "ImagesRec" in the working directory.

Note: the for stereo, the code has not been cleaned and the topics and path have to be set in the code.


# Calibration file (monocular)

The camera calibration and configuration are formated under the .yml format

```
%YAML:1.0
---
Topic: "/camera/image_color"
Camera_Matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 3.2804590995063145e+03, 0., 1.7114821106008048e+03, 0.,
       3.2922513808488702e+03, 1.5102827737530672e+03, 0., 0., 1. ]
Distortion_Coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ -1.6808745142290565e-01, 1.4822065370004972e-01,
       4.5216204666395374e-03, 1.2071351593942686e-03,
       3.9637280456063734e-03 ]
image_Width: 3376
image_Height: 2704
```
