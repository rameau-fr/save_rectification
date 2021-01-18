#include "monocam.h"

monoCam::monoCam()
{
}

monoCam::monoCam(Mat ImageT)
{
    Image = ImageT; 
    //GaussianBlur( Image, Image, Size( 3, 3 ), 0, 0 );
}

void monoCam::OpenParams(string ParametersFile) {

    //set up a FileStorage object to read camera params from file
    FileStorage fs;
    fs.open(ParametersFile, FileStorage::READ);

    // read cameras matrices, distortions
    fs["Camera_Matrix"] >> Intrinsics;
    fs["Distortion_Coefficients"] >> Distorsion;
    fs["image_Width"] >> imgSize.width;
    fs["image_Height"] >> imgSize.height;
    fs["Topic"] >> topic;
    // close the input file
    fs.release();
}


// Rectify stereo Images and save new parameters
void monoCam::RectifyImage(){

    // Init Rectification (if LUT is empty)
    if (rmap[0][0].empty()) 
    {
        Mat R1 ;
        Size SizeAfterRec = imgSize ; //same size as input

        // Rectification parameters
        Intrinsics_Rec = Intrinsics.clone();
        initUndistortRectifyMap(Intrinsics, Distorsion, R1, Intrinsics_Rec, SizeAfterRec, CV_16SC2 , rmap[0][0], rmap[0][1]);
        imgSize = SizeAfterRec;

        // Rectify images
        remap(Image, Image_Rec, rmap[0][0], rmap[0][1], INTER_LINEAR);
    }
    else
    {
        // Rectify images
        remap(Image, Image_Rec, rmap[0][0], rmap[0][1], INTER_LINEAR);
    }
}


void monoCam::InitRecParams(){

    // Init Rectification (if LUT is empty)
    Mat R1 ;
    Size SizeAfterRec = imgSize ;

    // Rectification parameters
    Intrinsics_Rec = Intrinsics.clone();
    initUndistortRectifyMap(Intrinsics, Distorsion, R1, Intrinsics_Rec, SizeAfterRec, CV_16SC2 , rmap[0][0], rmap[0][1]);
    imgSize = SizeAfterRec;

    // Display the values
    cout << "Intrinsic rectif :: " << Intrinsics_Rec <<endl;
    cout << "size after rectif :: " << SizeAfterRec <<endl;
   
}
