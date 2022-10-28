#include "tf.h"

namespace roswrapper {
TFPublisher::TFPublisher(const std::string& base_frame_id, const std::string& child_frame_id) : frame_id_(base_frame_id), child_id_(child_frame_id) {
  br_ = tf::TransformBroadcaster();
}

void TFPublisher::publish(KeyFrame& key_frame) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(key_frame.trans.x(), key_frame.trans.y(), key_frame.trans.z()));

  auto rpy = key_frame.rot.rpy();

  tf::Quaternion q;
  q.setRPY(rpy(0), rpy(1), rpy(2));
  transform.setRotation(q);

  br_.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(key_frame.time), frame_id_, child_id_));
}
}  // namespace roswrapper