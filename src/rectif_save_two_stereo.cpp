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
SLAMParamsClass *SLAMParams1;
SLAMParamsClass *SLAMParams2;
string CarName1; string TopicL1; string TopicR1;
string CarName2; string TopicL2; string TopicR2;
string DispPubName;

class StereoDisp 
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    // need to define publisher for specified message
    message_filters::Subscriber<sensor_msgs::Image> sub_imageL1;
    message_filters::Subscriber<sensor_msgs::Image> sub_imageR1;
    message_filters::Subscriber<sensor_msgs::Image> sub_imageL2;
    message_filters::Subscriber<sensor_msgs::Image> sub_imageR2;
	
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    void saveImage(const char* prefix, const cv::Mat& image);
    message_filters::Synchronizer< MySyncPolicy > sync;
	
    // image variable
    Mat imageL1, imageR1; Mat imageL2, imageR2;
    int save_count_=0;
    boost::format filename_format_;

public:
    StereoDisp() : it_(nh_), 
    filename_format_(""),
    sub_imageL1(nh_,"/" + TopicL1,15),
    sub_imageR1(nh_,"/" +TopicR1,15),
    sub_imageL2(nh_,"/" +TopicL2,15),
    sub_imageR2(nh_,"/" +TopicR2,15),
    sync( MySyncPolicy(500), sub_imageL1, sub_imageR1, sub_imageL2, sub_imageR2 )
    { 
        ROS_INFO("wait for data :)");
        sync.registerCallback( boost::bind( &StereoDisp::stereoImgCb, this, _1, _2, _3, _4) );
        std::string format_string;
		nh_.param("filename_format", format_string, std::string("%s%06i.png"));
		filename_format_.parse(format_string);
    }
    void stereoImgCb(const sensor_msgs::ImageConstPtr &msg_imageL1, const sensor_msgs::ImageConstPtr &msg_imageR1, const sensor_msgs::ImageConstPtr &msg_imageL2, const sensor_msgs::ImageConstPtr &msg_imageR2);
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
    

void StereoDisp::stereoImgCb(const sensor_msgs::ImageConstPtr &msg_imageL1, const sensor_msgs::ImageConstPtr &msg_imageR1, const sensor_msgs::ImageConstPtr &msg_imageL2, const sensor_msgs::ImageConstPtr &msg_imageR2 ){
    
	ROS_INFO("image L/R received :)");
	Mat Intrinsic_Rec1  = SLAMParams1->Intrinsics_Rec; 
	double baseline1 = SLAMParams1->baseline; 
  Mat Intrinsic_Rec2  = SLAMParams2->Intrinsics_Rec; 
  double baseline2 = SLAMParams2->baseline; 

	// ----- subscribe image L/R rig1
	cv_bridge::CvImagePtr cvPtr_imageL1;
	cv_bridge::CvImagePtr cvPtr_imageR1;
	cvPtr_imageL1 = cv_bridge::toCvCopy(msg_imageL1, sensor_msgs::image_encodings::BGR8);
	cvPtr_imageR1 = cv_bridge::toCvCopy(msg_imageR1, sensor_msgs::image_encodings::BGR8);
	imageL1 = cvPtr_imageL1->image;
	imageR1 = cvPtr_imageR1->image;

  // ----- subscribe image L/R rig2
  cv_bridge::CvImagePtr cvPtr_imageL2;
  cv_bridge::CvImagePtr cvPtr_imageR2;
  cvPtr_imageL2 = cv_bridge::toCvCopy(msg_imageL2, sensor_msgs::image_encodings::BGR8);
  cvPtr_imageR2 = cv_bridge::toCvCopy(msg_imageR2, sensor_msgs::image_encodings::BGR8);
  imageL2 = cvPtr_imageL2->image;
  imageR2 = cvPtr_imageR2->image;

	// ----- Rectify images rig1
	Mat ImageL_Rec1, ImageR_Rec1;
	remap(imageL1, ImageL_Rec1, SLAMParams1->rmap[0][0], SLAMParams1->rmap[0][1], INTER_LINEAR);
	remap(imageR1, ImageR_Rec1, SLAMParams1->rmap[1][0], SLAMParams1->rmap[1][1], INTER_LINEAR);
	ImageL_Rec1.copyTo(imageL1); ImageR_Rec1.copyTo(imageR1);

    // ----- Rectify images rig2
  Mat ImageL_Rec2, ImageR_Rec2;
  remap(imageL2, ImageL_Rec2, SLAMParams2->rmap[0][0], SLAMParams1->rmap[0][1], INTER_LINEAR);
  remap(imageR2, ImageR_Rec2, SLAMParams2->rmap[1][0], SLAMParams1->rmap[1][1], INTER_LINEAR);
  ImageL_Rec2.copyTo(imageL2); ImageR_Rec2.copyTo(imageR2);

	// --------------------- Write the rectified images
	if (!ImageL_Rec1.empty())
	{
    char tmp_char[] = "./ImagesLC1/camera_Left_";
    saveImage(tmp_char,ImageL_Rec1);
   }
  if (!ImageR_Rec1.empty())
  {
    char tmp_char_rcv[] = "./ImagesRC1/camera_Right_";
    saveImage(tmp_char_rcv,ImageR_Rec1);
  }
  if (!ImageL_Rec2.empty())
  {
    char tmp_char[] = "./ImagesLC2/camera_Left_";
    saveImage(tmp_char,ImageL_Rec2);
  }
  if (!ImageR_Rec2.empty())
  {
    char tmp_char_rcv[] = "./ImagesRC2/camera_Right_";
    saveImage(tmp_char_rcv,ImageR_Rec2);
    save_count_++;
  }
    
}



int main(int argc, char **argv){
    
    	// ******** Open parameters of the first rig *****///
  string pathToCamParam1 = "/home/francois/Documents/Dataset/KAIST/IndoorPeople/StereoParams_rig_Bosch1.yml";
	SLAMParams1 = new SLAMParamsClass(pathToCamParam1);
 cout<< "parameters 1 found " << endl;
	FileStorage fs1; //set up a FileStorage object to read camera params from file
	fs1.open( pathToCamParam1, FileStorage::READ);
	fs1["CarName"] >> CarName1; fs1["ImageTopicL"] >> TopicL1; fs1["ImageTopicR"] >> TopicR1;
	fs1.release(); // close the input file

  // ******** Open parameters of the second rig *****///
  string pathToCamParam2 = "/home/francois/Documents/Dataset/KAIST/IndoorPeople/StereoParams_rig_Bosch2.yml";
  SLAMParams2 = new SLAMParamsClass(pathToCamParam2);
 cout<< "parameters 2 found " << endl;
  FileStorage fs2; //set up a FileStorage object to read camera params from file
  fs2.open( pathToCamParam2, FileStorage::READ);
  fs2["CarName"] >> CarName2; fs2["ImageTopicL"] >> TopicL2; fs2["ImageTopicR"] >> TopicR2;
  fs2.release(); // close the input file
    
	ros::init(argc, argv, "SaveStereoRectif");
	StereoDisp DisparityComp;
	ros::spin();
}


