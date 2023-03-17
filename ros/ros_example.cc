// cpptemplates
#include "app/app_example.h"
#include "cpptemplates/DParamConfig.h"
#include "cpptemplates/demo.h"
#include "cpptemplates/func.h"
#include "reflcpp/core.hpp"
#include "reflcpp/yaml.hpp"
#include "tools/rosbag.h"
// ros
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
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

bool srv_callback(cpptemplates::funcRequest& req, cpptemplates::funcResponse& res) {
  res.result = app->service(req.arg0, req.arg1);
  return true;
}

void dcfg_callback(cpptemplates::DParamConfig& cfg, uint32_t /*level*/) {
  app->feed_dcfg(cfg.int_param, cfg.double_param, cfg.str_param);
}

/****************************************/
/*        program configs struct        */
/****************************************/
struct ProgramConfigs {
  // rosbag
  std::string bag_file;
  float bag_rate;
  // app configuration file path
  std::string app_config;
};
REFLCPP_METAINFO(ProgramConfigs, , (bag_file)(bag_rate))
REFLCPP_YAML(ProgramConfigs)

void as_absolute_path(std::string &path, const std::string &prefix) {
  if(!path.empty() && fs::path(path).is_relative()) {
    path = fs::path(prefix) / path;
  }
}

int main(int argc, char* argv[]) {
  /****************************************/
  /*      configure from command line     */
  /****************************************/
  if (argc != 2) {
    ROS_FATAL_STREAM("usage: " << argv[0] << " CONFIG_FILE.");
    return -1;
  }
  ROS_INFO_STREAM("configuring from " << argv[1]);
  auto prog_cfg = YAML::LoadFile(argv[1]).as<ProgramConfigs>();
  auto prefix_path = fs::path(argv[1]).parent_path();
  as_absolute_path(prog_cfg.bag_file, prefix_path);
  as_absolute_path(prog_cfg.app_config, prefix_path);
  /****************************************/
  /*            configure app             */
  /****************************************/
  app = std::make_unique<AppExample>(prog_cfg.app_config);

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
  ros::Publisher rst_stamp_pub = nh.advertise<cpptemplates::demo>("/rst_stamp", 1);

  // dynamic reconfigure here
  dynamic_reconfigure::Server<cpptemplates::DParamConfig> dcfg_server;
  dcfg_server.setCallback(&dcfg_callback);
  // service here
  ros::ServiceServer srv = nh.advertiseService("/demo_srv", srv_callback);

  /****************************************/
  /*            configure bag             */
  /****************************************/
  BagPlayer bag_player(nh);
  if (!prog_cfg.bag_file.empty()) {
    bag_player.open(prog_cfg.bag_file);
    bag_player.set_queue_size(1);
    bag_player.set_rate(prog_cfg.bag_rate);  // set to 0 for full speed (not recommend)
  }

  /****************************************/
  /*              main loop               */
  /****************************************/
  ROS_INFO_STREAM("begin loop.");
  auto callbacks = ros::getGlobalCallbackQueue();
  while (ros::ok()) {
    // play rosbag
    if (bag_player.is_open()) {
      // end loop if bag end
      if (bag_player.eof())
        ros::shutdown();
      bag_player.play_once();
    }
    // process ros message callback (sleep forever if no callbacks available)
    if(bag_player.is_open()) {
      callbacks->callAvailable();
    } else {
      callbacks->callAvailable(ros::WallDuration(999));
    }    
  }

  /****************************************/
  /*    (optional): some post process     */
  /****************************************/

  return 0;
}
