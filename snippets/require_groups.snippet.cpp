#include <ros/ros.h>
#include <std_msgs/String.h>

#include <ros_parameter/ros_parameter.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_node");

  ros::NodeHandle n;

  ros_parameter::ParameterGroup node_pg("~");

  ros_parameter::Parameter<std::string> msg1;
  // The first parameter is the leaf of the message name, and the second is the default value
  msg1 = node_pg.create_parameter("msg1", "Hello");

  // Then to create ~controls/kp, for example, you would create another group:
  ros_parameter::ParameterGroup controls_pg;
  controls_pg = node_pg.create_parameter_group("controls"); // Namespace for the group
  // Then create the parameter
  ros_parameter::Parameter<double> kp;
  kp = controls_pg.create_parameter("kp", 1.0);
  // This would be currently achieved like this:
  // ros_parameter::Parameter<double> kp("~controls/kp", 1.0);

  // Then you can optionally attach callbacks at any level
  // node_pg.on_update would get called anytime any parameters in the node's namespace changed
  // msg1.on_update would get called only when ~msg1 is updated
  // controls_pg.on_update would get called only when a parameter in the ~controls namespace changes
  // kp.on_update would get called only when ~controls/kp gets updated

  ros::spin();

  return 0;
}
