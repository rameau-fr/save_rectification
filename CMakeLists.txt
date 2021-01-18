cmake_minimum_required(VERSION 2.8.3)
project(rectif_save)

add_definitions(-std=c++11)  
set(CMAKE_BUILD_TYPE Release)  
set(DCMAKE_BUILD_TYPE Release) 
set(DROS_BUILD_TYPE Release) 

find_package(catkin REQUIRED COMPONENTS  
cv_bridge  
image_transport  
message_filters  
roscpp  
sensor_msgs  
std_msgs  
stereo_msgs  
image_geometry  
image_proc  
message_generation  
)  

add_message_files(
   FILES
dum.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)
  
catkin_package(  
INCLUDE_DIRS include  
DEPENDS Eigen  
CATKIN_DEPENDS message_runtime
)  
  
  
include_directories(  
${catkin_INCLUDE_DIRS}  
include  
/usr/include/opencv  
)  
  
add_executable(rectif_save src/rectif_save.cpp  
include/SLAMParamsClass.h  )  
  
target_link_libraries(rectif_save  
 -L/usr/local/lib ${OpenCV_LIBS}
${catkin_LIBRARIES}  
)

add_executable(rectif_save_two_stereo src/rectif_save_two_stereo.cpp  
include/SLAMParamsClass.h  )  
  
target_link_libraries(rectif_save_two_stereo  
 -L/usr/local/lib ${OpenCV_LIBS} 
${catkin_LIBRARIES}  
)

add_executable(rectif_mono src/rectif_mono.cpp
src/monocam.cpp
include/monocam.h )  
  
target_link_libraries(rectif_mono  
 -L/usr/local/lib ${OpenCV_LIBS}
${catkin_LIBRARIES}  
)



