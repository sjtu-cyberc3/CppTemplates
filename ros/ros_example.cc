// app
#include "app/app_example.h"
// user
#include "cpptemplates2/demo.h"
#include "tools/rosbag.h"
// ros
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
  ros::Publisher rst_pub       = nh.advertise<std_msgs::String>("/rst", 1);
  ros::Publisher rst_stamp_pub = nh.advertise<cpptemplates2::demo>("/rst_stamp", 1);
  /****************************************/
  /*            configure bag             */
  /****************************************/
  BagPlayer bag_player(nh);
  if (!bag_file.empty()) {
    bag_player.open(bag_file);
    bag_player.set_queue_size(1);
    bag_player.set_rate(1);  // set to 0 for full speed (not recommend)
  }

  /****************************************/
  /*              main loop               */
  /****************************************/
  ROS_INFO_STREAM("begin loop.");
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    // play rosbag
    if (bag_player.is_open()) {
      // end loop if bag end
      if (bag_player.eof())
        ros::shutdown();
      bag_player.play_once();
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

      cpptemplates2::demo rst_msg_stamp;
      rst_msg_stamp.str          = rst_msg->data;
      rst_msg_stamp.header.stamp = ros::Time::now();
      rst_stamp_pub.publish(rst_msg_stamp);
    }
    // only sleep without rosbag
    if (!bag_player.is_open()) {
      loop_rate.sleep();
    }
  }

  /****************************************/
  /*    (optional): some post process     */
  /****************************************/

  return 0;
}