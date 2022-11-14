// app
#include "app/app_example.h"
// user
#include "ros/init.h"
#include "ros/publisher.h"
#include "tools/rosbag.h"
// ros
#include <iterator>
#include <optional>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
// standard c++
#include <memory>

/****************************************/
/*            user app here             */
/****************************************/
// lazy init to avoid unexpected problems
std::unique_ptr<AppExample> app;

/****************************************/
/*            callback here             */
/*    constptr for better performance   */
/****************************************/
void str_callback(std_msgs::StringConstPtr str_msg) {
  app->feed_str(str_msg->data);
}
void int_callback(std_msgs::Int32ConstPtr int_msg) {
  app->feed_int(int_msg->data);
}

int main(int argc, char* argv[]) {
  /****************************************/
  /*      configure from command line     */
  /****************************************/
  std::string config_file;
  std::string bag_file;
  if (argc <= 1) {
    std::cout << "usage: " << argv[0] << " config_file [bag_file]" << std::endl;
    return -1;
  }
  if (argc >= 2) {
    config_file = argv[1];
  }
  if (argc >= 3) {
    bag_file = argv[2];
  }
  ROS_INFO_STREAM("loading config from '" << config_file << "'.");
  if (bag_file.empty()) {
    ROS_INFO_STREAM("no bag file, run in native ros.");
  } else {
    ROS_INFO_STREAM("loading rosbag from '" << bag_file << "'.");
  }

  /****************************************/
  /*            configure app             */
  /****************************************/
  app = std::make_unique<AppExample>(config_file);

  /****************************************/
  /*            configure ros             */
  /****************************************/
  ros::init(argc, argv, "ros_example");
  ros::NodeHandle nh;

  // subscriber here
  ros::Subscriber str_sub = nh.subscribe("/str", 1, str_callback);
  ros::Subscriber int_sub = nh.subscribe("/int", 1, int_callback);

  // publisher here
  ros::Publisher rst_pub = nh.advertise<std_msgs::String>("/rst", 1);

  /****************************************/
  /*            configure bag             */
  /****************************************/
  bag<std_msgs::String, std_msgs::Int32> bag("/str", "/int");
  if (!bag_file.empty()) {
    bag.open(bag_file, nh);
  }

  /****************************************/
  /*              main loop               */
  /****************************************/
  ROS_INFO_STREAM("begin loop.");
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    // play rosbag
    if(bag.is_open()) {
      // end loop if bag end
      if(bag.eof()) ros::shutdown();
      bag.play_once();
    }
    // process ros message callback
    ros::spinOnce();
    // process user algorithm
    app->process();
    // process user result
    auto result = app->get_rst();
    if (result.has_value()) {
      std_msgs::StringPtr rst_msg(new std_msgs::String);
      rst_msg->data = std::move(result.value());
      rst_pub.publish(rst_msg);
    }
    // sleep
    loop_rate.sleep();
  }

  /****************************************/
  /*    (optional): some post process     */
  /****************************************/

  return 0;
}