#include <MarbleVirtualComms.h>
#define USAGE "\nUSAGE: marble_virtual_comms\n"

using namespace marble_virtual_comms;

int main(int argc, char** argv){
  ros::init(argc, argv, "marble_virtual_comms");
  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");

  if (argc == 2 && std::string(argv[1]) == "-h") {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  MarbleVirtualComms mvc(private_nh, nh);

  try {
    ros::spin();
  } catch (std::runtime_error& e) {
    ROS_ERROR("marble_virtual_comms exception: %s", e.what());
    return -1;
  }

  return 0;
}
