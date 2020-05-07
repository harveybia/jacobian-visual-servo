/*
 * Camera info publisher for april tag
 * Henry Zhang
 */

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "image_transport/publisher.h"
#include "ros/init.h"

struct MyCamInfo {
  int height;
  int width;

  double K[9];
  double P[12];
};  // struct MyCamInfo

// publish a camera info everytime we got a frame of image
void image_cb(const sensor_msgs::ImageConstPtr &image_ptr, ros::Publisher &pub,
              const MyCamInfo &my_cam_info) {
  // create a camera info msg
  sensor_msgs::CameraInfo camera_info;
  camera_info.header = image_ptr->header;
  camera_info.height = my_cam_info.height;
  camera_info.width = my_cam_info.width;
  for (int i = 0; i < 9; i++) {
    camera_info.K[i] = my_cam_info.K[i];
  }
  for (int i = 0; i < 12; i++) {
    camera_info.P[i] = my_cam_info.P[i];
  }

  // publish
  pub.publish(camera_info);
}

int main(int argc, char **argv) {
  // setup ros
  ros::init(argc, argv, "camera_info_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  image_transport::ImageTransport it(nh);

  // setup const camera info
  MyCamInfo my_cam_info;
  pnh.getParam("img_height", my_cam_info.height);
  pnh.getParam("img_width", my_cam_info.width);
  pnh.getParam("fx", my_cam_info.K[0]);
  pnh.getParam("fy", my_cam_info.K[4]);
  pnh.getParam("cx", my_cam_info.K[2]);
  pnh.getParam("cy", my_cam_info.K[5]);
  my_cam_info.P[0] = my_cam_info.K[0];  // fx
  my_cam_info.P[5] = my_cam_info.K[4];  // fy
  my_cam_info.P[2] = my_cam_info.K[2];  // cx
  my_cam_info.P[6] = my_cam_info.K[5];  // cy

  // setup subscriber and publisher
  std::string image_topic;
  pnh.getParam("image_topic", image_topic);
  ROS_INFO_STREAM("image_topic: " << image_topic);
  std::string camera_info_topic =
      image_topic.substr(
          0, image_topic.substr(1, image_topic.length()).find('/') + 1) +
      "/camera_info";
  ROS_INFO_STREAM("camera_info_topic: " << camera_info_topic);

  ros::Publisher pub =
      nh.advertise<sensor_msgs::CameraInfo>(camera_info_topic, 1);
  image_transport::Subscriber sub =
      it.subscribe(image_topic, 1, boost::bind(image_cb, _1, pub, my_cam_info));

  ros::spin();

  return 0;
}
