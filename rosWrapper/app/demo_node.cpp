#include "demo_node.h"

namespace cyberc3 {
namespace demo {}  // namespace demo
}  // namespace cyberc3

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo");
  ros::NodeHandle          nh_("~");
  cyberc3::demo::demo_node demo_node_(nh_);

  ros::spin();

  return 0;
}
