#include <stdio.h>
#include <string>
#include <ctime>
#include <iostream>
#include <fstream>
#include <limits>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/scoped_ptr.hpp>
#include <boost/function.hpp>
#include <boost/format.hpp>

#include <opencv2/opencv.hpp>

// calib
#include <SLAMParamsClass.h>

// finding path
#include <termios.h>
#include <pwd.h>

using namespace cv;
using namespace std;

using namespace sensor_msgs;
using namespace message_filters;

// Global camera Params
SLAMParamsClass *SLAMParams;
string CarName; string TopicL; string TopicR;
string DispPubName;

class StereoDisp 
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    // need to define publisher for specified message
    message_filters::Subscriber<sensor_msgs::Image> sub_imageL;
    message_filters::Subscriber<sensor_msgs::Image> sub_imageR;
	
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    void saveImage(const char* prefix, const cv::Mat& image);
    message_filters::Synchronizer< MySyncPolicy > sync;
	
    // image variable
    Mat imageL, imageR;
	int save_count_=0;
	boost::format filename_format_;

public:
    StereoDisp() : it_(nh_), 
    filename_format_(""),
    sub_imageL(nh_,"/" + TopicL,5),
    sub_imageR(nh_,"/" +TopicR,5),
    sync( MySyncPolicy(100), sub_imageL, sub_imageR)
    { 
        ROS_INFO("wait for data :)");
        sync.registerCallback( boost::bind( &StereoDisp::stereoImgCb, this, _1, _2) );
        std::string format_string;
		nh_.param("filename_format", format_string, std::string("%s%06i.png"));
		filename_format_.parse(format_string);
    }
    void stereoImgCb(const sensor_msgs::ImageConstPtr &msg_imageL, const sensor_msgs::ImageConstPtr &msg_imageR);
};

void StereoDisp::saveImage(const char* prefix, const cv::Mat& image)
  {
	

    if (!image.empty()) {
      std::string filename = (filename_format_ % prefix % save_count_).str();
      vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);
      cv::imwrite(filename, image, compression_params);
      ROS_INFO("Saved image %s", filename.c_str());
    } else {
      ROS_WARN("Couldn't save %s image, no data!", prefix);
    }
  }
    

void StereoDisp::stereoImgCb(const sensor_msgs::ImageConstPtr &msg_imageL, const sensor_msgs::ImageConstPtr &msg_imageR){
    
	ROS_INFO("image L/R received :)");
	Mat Intrinsic_Rec  = SLAMParams->Intrinsics_Rec; 
	double baseline = SLAMParams->baseline; 

	// ----- subscribe image L/R    
	cv_bridge::CvImagePtr cvPtr_imageL;
	cv_bridge::CvImagePtr cvPtr_imageR;
	cvPtr_imageL = cv_bridge::toCvCopy(msg_imageL, sensor_msgs::image_encodings::BGR8);
	cvPtr_imageR = cv_bridge::toCvCopy(msg_imageR, sensor_msgs::image_encodings::BGR8);
	imageL = cvPtr_imageL->image;
	imageR = cvPtr_imageR->image;

	// ----- Rectify images
	Mat ImageL_Rec, ImageR_Rec;
	remap(imageL, ImageL_Rec, SLAMParams->rmap[0][0], SLAMParams->rmap[0][1], INTER_LINEAR);
	remap(imageR, ImageR_Rec, SLAMParams->rmap[1][0], SLAMParams->rmap[1][1], INTER_LINEAR);
	ImageL_Rec.copyTo(imageL); ImageR_Rec.copyTo(imageR);

	// --------------------- Write the rectified images
	if (!ImageL_Rec.empty())
	{
	char tmp_char[] = "./ImagesLC/camera_Left_";
     saveImage(tmp_char,ImageL_Rec);
     }
    if (!ImageR_Rec.empty())
    {
	char tmp_char_rcv[] = "./ImagesRC/camera_Right_";
      saveImage(tmp_char_rcv,ImageR_Rec);
      save_count_++;
    }
    
}



int main(int argc, char **argv){
    
    	// Open parameters
	//string pathToCamParam = "/Documents/DataBosch19Dec2018/car1_h22_white/car2_h17_blue/SLAMParams.yml";
  string pathToCamParam = "/home/francois/Documents/Dataset/KAIST/IndoorPeople/StereoParams_rig_Bosch1.yml";
	SLAMParams = new SLAMParamsClass(pathToCamParam);
 cout<< "parameters found " << endl;
	// Read the poses of teh connected cars
	FileStorage fs; //set up a FileStorage object to read camera params from file
	fs.open( pathToCamParam, FileStorage::READ);
	fs["CarName"] >> CarName; fs["ImageTopicL"] >> TopicL; fs["ImageTopicR"] >> TopicR;
	fs.release(); // close the input file
    
	ros::init(argc, argv, "SaveStereoRectif");
	StereoDisp DisparityComp;
	ros::spin();
}


