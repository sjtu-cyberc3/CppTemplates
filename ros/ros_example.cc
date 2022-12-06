// app
#include "app/app_example.h"
// user
#include "common/configs/configs.hpp"
#include "cpptemplates2/DParamConfig.h"
#include "cpptemplates2/demo.h"
#include "cpptemplates2/func.h"
#include "tools/rosbag.h"
// ros
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
// standard c++
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

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
bool srv_callback(cpptemplates2::funcRequest& req, cpptemplates2::funcResponse& res) {
  res.result = app->service(req.arg0, req.arg1);
  return true;
}

void dcfg_callback(cpptemplates2::DParamConfig& cfg, uint32_t /*level*/) {
  app->feed_dcfg(cfg.int_param, cfg.double_param, cfg.str_param);
}

int main(int argc, char* argv[]) {
  /****************************************/
  /*      configure from command line     */
  /****************************************/
  std::string config_file_path = fs::path(WORK_SPACE_PATH) / "config/ros_example.yaml";
  Configs     cfg;
  ConfigDef(cfg, std::string, bag_file);
  ConfigDef(cfg, std::string, AppExampleConfig);
  cfg.Open(config_file_path);
  cfg.LoadOnce();
  /****************************************/
  /*            configure app             */
  /****************************************/
  app = std::make_unique<AppExample>(AppExampleConfig);

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

  // dynamic reconfigure here
  dynamic_reconfigure::Server<cpptemplates2::DParamConfig> dcfg_server;
  dcfg_server.setCallback(&dcfg_callback);
  // service here
  ros::ServiceServer srv = nh.advertiseService("/demo_srv", srv_callback);

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
