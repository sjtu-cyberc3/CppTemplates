#include "rosbag.h"

BagPlayer::BagPlayer(ros::NodeHandle nh) : nh_(nh) {}

bool BagPlayer::open(const std::string& filename, std::vector<std::string> topics) {
  bag_.open(filename, rosbag::bagmode::Read);
  if (!bag_.isOpen())
    return false;
  if (topics.empty()) {
    view_.addQuery(bag_);
  } else {
    view_.addQuery(bag_, rosbag::TopicQuery(topics));
  }
  view_it_  = view_.begin();
  view_end_ = view_.end();
  return true;
}

bool BagPlayer::is_open() const {
  return bag_.isOpen();
}

void BagPlayer::set_queue_size(int size) {
  queue_size_ = size;
}

int BagPlayer::get_queue_size() const {
  return queue_size_;
}

void BagPlayer::set_rate(float rate) {
  rate_ = rate;
}

int BagPlayer::get_rate() const {
  return rate_;
}

bool BagPlayer::eof() const {
  return view_it_ == view_end_;
}

void BagPlayer::play_once() {
  if (eof())
    return;
  auto      msg               = *(view_it_++);
  ros::Time curr_msg_bag_time = msg.getTime();
  ros::Time curr_msg_pub_time;

  static bool first_play = true;
  if (first_play) {
    curr_msg_pub_time = ros::Time::now();
    first_play        = false;
  } else if (rate_ > 0) {
    curr_msg_pub_time = prev_msg_pub_time_ + (curr_msg_bag_time - prev_msg_bag_time_) * (1 / rate_);
    ros::Time::sleepUntil(curr_msg_pub_time);
  }

  std::string topic  = msg.getTopic();
  auto        pub_it = pubs_.find(topic);
  if (pub_it == pubs_.end()) {
    ros::AdvertiseOptions opts(topic, queue_size_, msg.getMD5Sum(), msg.getDataType(), msg.getMessageDefinition());
    pub_it = pubs_.emplace(topic, nh_.advertise(opts)).first;
  }
  pub_it->second.publish(msg);

  prev_msg_bag_time_ = curr_msg_bag_time;
  prev_msg_pub_time_ = curr_msg_pub_time;
}
