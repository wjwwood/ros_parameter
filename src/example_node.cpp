#include <ros/ros.h>
#include <std_msgs/String.h>

#include <ros_parameter/ros_parameter.h>

ros::Publisher pub;

// Right now it returns void, but you could also have a function which returns a
// std::map<string, bool> which maps parameter names to True if the change is
// accepted, False if it is rejected This allows the user's code to reject
// the proposed parameter change. You could imagine that you do the change
// acceptance based on state of pg after the function returns, i.e. if the
// parameter values are unchanged by the callback, then the user has accepted
// them. But this would prevent the user from rejecting a value which received
// an update, yet was not changed.
void on_group_change(ros_parameter::ParameterGroup &pg, std::map<std::string, bool> &changed)
{
    if (changed["~msg1"] && changed["~msg2"]) {
        ROS_INFO("Both changed.");
    } else if (changed["~msg1"]) {
        ROS_INFO("~msg1 changed");
    } else if (changed["~msg2"]) {
        ROS_INFO("~msg2 changed");
    } else {
        ROS_INFO("Neither changed? This shouldn't happen.");
    }
    if (changed["~msg1"] || changed["~msg2"]) {
        std_msgs::String msg;
        // I don't really like this part of the API, see if you can come up with something better
        std::string msg1;
        pg.get_data("~msg1", msg1);
        std::string msg2;
        pg.get_data("~msg1", msg2);
        msg.data = "Message: " + msg1 + " " + msg2 + "!";
        pub.publish(msg);
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_node");

  ros::NodeHandle n;

  pub = n.advertise<std_msgs::String>("output", 1000);

  // I like this simplified parameter templatization, we should hide the caching options
  // It might even be an option to drop the templating all together and do inheritance
  ros_parameter::Parameter<std::string> msg1("~msg1", "Hello");
  // If I wanted to make msg1 static, I would like to do it like this:
  //   msg1.make_static();
  // Or using a subclass:
  //   ros_parameter::StaticParameter<std::string> msg1(...);
  ros_parameter::Parameter<std::string> msg2("~msg2", "World");

  ros_parameter::ParameterGroup pg;
  // This is nice because the templating of add_parameter is inferred
  pg.add_parameter(msg1);
  pg.add_parameter(msg2);

  // There may be a distinction between on change and on update
  pg.on_change(on_group_change);

  ros::spin();

  return 0;
}
