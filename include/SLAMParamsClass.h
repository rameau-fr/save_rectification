#ifndef SLAMParams_H
#define SLAMParams_H

#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


class SLAMParamsClass{
public:
    SLAMParamsClass();//constructor
    ~SLAMParamsClass() {};//destructor

    // Camera Parameters
    Mat IntrinsicsL, DistorsionL, IntrinsicsR, DistorsionR, Stereo_Rotation, Stereo_Translation;
    Size Stereo_imgSize;
    Size SizeAfterRec;
    Mat R1,R2,P1,P2;
    Mat rmap[2][2]; // Rectif LUT
    bool rectif_method;// 0 = already rectified images, 1= full rectif
    int nbPts;
    int fast_ratio_max, fast_ratio_min;
    int ANMS_Method;
    double ANMS_tolerance;
    double min_dist;
    int max_disp;
    int ytol;
    int Radius;
    int Ransac_thresh;
    int KFinLocal;
	Mat Q;
    // Rectified Parameters
    Mat Intrinsics_Rec;
    double baseline;

    SLAMParamsClass(string path):pathInt(path){
        FileStorage fs; //set up a FileStorage object to read camera params from file
        fs.open(pathInt, FileStorage::READ);

        // read cameras matrices, distortions and extrinsic coefficients from file
        fs["Camera_MatrixL"] >> IntrinsicsL;
        fs["Distortion_CoefficientsL"] >> DistorsionL;
        fs["Camera_MatrixR"] >> IntrinsicsR;
        fs["Distortion_CoefficientsR"] >> DistorsionR;
        fs["Stereo_Rotation"] >> Stereo_Rotation;
        fs["Stereo_Translation"] >> Stereo_Translation;
        fs["image_Width"] >> Stereo_imgSize.width;
        fs["image_Height"] >> Stereo_imgSize.height;
        fs["baseline"] >> baseline;
        fs["Rectif.bool"] >> rectif_method;
        fs["Rectif.width"] >> SizeAfterRec.width;
        fs["Rectif.heigh"] >> SizeAfterRec.height;
        fs["numPts"] >> nbPts;
        fs["fast_ratio_min"] >> fast_ratio_min;
        fs["fast_ratio_max"] >> fast_ratio_max;
        fs["ANMS_method"] >> ANMS_Method;
        fs["ANMS_tolerance"] >> ANMS_tolerance;
        fs["ytol"] >> ytol;
        fs["min_dist"] >> min_dist;
        fs["Radius"] >> Radius;
        fs["Ransac_thresh"] >> Ransac_thresh;
        fs["KFinLocal"] >> KFinLocal;
        fs.release(); // close the input file

	
        //* Initialize image rectification
        
        if (rectif_method == false) // Already rectified images
        {
            // Params
            Intrinsics_Rec = IntrinsicsL;
            baseline = abs(baseline);
        }
        else if (rectif_method == true) // Rectification
        {
            // Compute Rectification Look Up table
            stereoRectify(IntrinsicsL, DistorsionL, IntrinsicsR, DistorsionR, Stereo_imgSize, Stereo_Rotation, Stereo_Translation, R1, R2, P1, P2, Q,CV_CALIB_ZERO_DISPARITY, 0, SizeAfterRec);
            initUndistortRectifyMap(IntrinsicsL, DistorsionL, R1, P1, SizeAfterRec, CV_16SC2 , rmap[0][0], rmap[0][1]); //Left
            initUndistortRectifyMap(IntrinsicsR, DistorsionR, R2, P2, SizeAfterRec, CV_16SC2 , rmap[1][0], rmap[1][1]); //Right

            // Save new parameters
            Intrinsics_Rec = P1(cv::Range(0,3), cv::Range(0,3));
            Mat Temp = Intrinsics_Rec.inv()*P2(cv::Range(0,3), cv::Range(3,4));
            baseline = abs(Temp.at<double>(0,0));
            cout << "Intrinsic Rec " << Intrinsics_Rec << endl;
		cout << "baseline " << baseline << endl;
        }

        //Compute max_disparity value
        max_disp = (int)round((Intrinsics_Rec.at<double>(0,0)*baseline)/min_dist);

    }

private:
    string pathInt;
};

#endif // SLAMParams_H
