#pragma once

// standard c++
#include <map>
#include <string>
#include <vector>
// ros
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

class BagPlayer {
 public:
  explicit BagPlayer(ros::NodeHandle nh);

  bool open(const std::string& filename, std::vector<std::string> topics = {});

  bool is_open() const;

  void set_queue_size(int size);

  int get_queue_size() const;

  void set_rate(float rate);

  int get_rate() const;

  bool eof() const;

  void play_once();

 private:
  ros::NodeHandle nh_;

  std::map<std::string, ros::Publisher> pubs_;
  rosbag::Bag                           bag_;
  rosbag::View                          view_;
  rosbag::View::const_iterator          view_it_;
  rosbag::View::const_iterator          view_end_;

  ros::Time prev_msg_bag_time_;
  ros::Time prev_msg_pub_time_;

  int queue_size_ = 0;

  float rate_ = 1;  // 0 for full speed
};
