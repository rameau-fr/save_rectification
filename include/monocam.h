#ifndef REARCAR_H
#define REARCAR_H

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

class monoCam
{
public:
    ~monoCam() {}
    Mat Image, Image_Rec;
    Mat rmap[2][2]; // Rectif LUT

    // Camera Parameters
    Mat Intrinsics, Intrinsics_Rec, Distorsion; Size imgSize;

   // Topic
   string topic;

    // Function
    monoCam();
    monoCam(Mat ImageT); // Declare with image
    void OpenParams (string ParametersFile); // Open camera's parameters
    void RectifyImage(); // Image rectif
    void InitRecParams();
};

#endif // REARCAR_H
