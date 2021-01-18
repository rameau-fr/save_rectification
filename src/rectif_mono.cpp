#include <stdio.h>
#include <string>
#include <ctime>
#include <iostream>
#include <fstream>
#include <limits>

#include <boost/format.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <opencv2/opencv.hpp>

// calib
#include <monocam.h>

// finding path
#include <termios.h>
#include <pwd.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

// Global camera Params
monoCam camera;
int countim = 0;

void saveImage(const char* prefix, const cv::Mat& image, int count)
{
  if (!image.empty()) {
    std::string format_string = "%s%06i.png";
    boost::format filename_format_;
    filename_format_.parse(format_string);
    std::string filename = (filename_format_ % prefix % count).str();
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(9);
    cv::imwrite(filename, image, compression_params);
    ROS_INFO("Saved image %s", filename.c_str());
  } else {
    ROS_WARN("Couldn't save %s image, no data!", prefix);
  }
}

// Synch images and tracks
void ImageCallBack( const ImageConstPtr& msgRGB)
{
	ROS_INFO("receive the image");

	// Extract RGB image
	Mat rgb_image = cv_bridge::toCvShare(msgRGB, "bgr8")->image;

  // Rectify image
  rgb_image.copyTo(camera.Image);
  camera.RectifyImage();
  Mat ImRec; camera.Image_Rec.copyTo(ImRec);

  // Display
  cv::imshow("out", ImRec);
  char key = (char)cv::waitKey(1);

  //  Write the rectified image
	if (!ImRec.empty())
	{
    char tmp_char[] = "./ImagesRec/Image_rec_";
    saveImage(tmp_char,ImRec, countim);
    countim++;
   }
}

int main(int argc, char **argv){
  
  ros::init(argc, argv, "Rectify_mono");
  ros::NodeHandle nh("~");

  // path to the intrinsic
  string filename; // = "/home/francois/catkin_ws/src/rectif_save/Intrinsics.yaml";
	if (nh.getParam("Path_Intrinsic", filename)){ }
  cout << "intrinsic path :: " << filename << endl;
  // Load intrinsic parameters
  cout << endl << "Reading Intrinsics" << endl;
  
  camera.OpenParams(filename);
  camera.InitRecParams();

  // message topic name
	std::string image_topic = camera.topic;
	
	// Subscribe to two input topics.
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe(image_topic, 10, ImageCallBack);

  //Spin it
  ros::spin();
}


