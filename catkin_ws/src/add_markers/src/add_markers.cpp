#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <string.h>

#include "CDeliverySpec.h"

//================================= main() ====================
int main(int argc, char **argv) {
  ros::init(argc, argv, "add_markers");

  geometry_msgs::Pose posePickup = createPose(1.0, -3.0);
  geometry_msgs::Pose poseDropoff = createPose(-2.0, 0.0);

  CDeliverySpec deliverySpec;
  deliverySpec.addDelivery(posePickup, poseDropoff);
  deliverySpec.start();

  ros::spin(); // Keep thread alive to service callbacks

  return 0;
}
