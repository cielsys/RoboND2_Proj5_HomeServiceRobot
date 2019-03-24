#include <ros/ros.h>
#include <math.h>
#include <string.h>
#include <cmath>
#include <sstream>

#include "CDeliverySpec.h"

//=============================================================
//================================= CDeliverySpec =============

//================================= ctor() ====================
CDeliverySpec::CDeliverySpec() : actionClientMoveBase("move_base", true) {
  init();
}

//================================= init() ====================
void CDeliverySpec::init() {
  marker = createMarker();
  poseRobotHome = createPose(0, 0);

  ros::NodeHandle hNode;
  pubMarker = hNode.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  subOdom = hNode.subscribe("/odom", 1000, &CDeliverySpec::cbOdom, this);
  //subOdom = hNode.subscribe("/odom", 1000, &CDeliverySpec::cbOdom);
  //subOdom = hNode.subscribe("/odom", 1000, boost::bind(&CDeliverySpec::cbOdom, this, _1));

  while (pubMarker.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  while (!actionClientMoveBase.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for MoveBase server...");
  }
}

//================================= addDelivery() ====================
void CDeliverySpec::addDelivery(geometry_msgs::Pose posePickup, geometry_msgs::Pose poseDropoff) {
  setDeliveryState(eDeliveryState::IDLE);

  this->posePickup = posePickup;
  this->poseDropoff = poseDropoff;
  setMarker(posePickup);
  showMarker(true);
  //moveToGoal(posePickup);
}

//================================= showMarker() ====================
void CDeliverySpec::showMarker(bool doShow) {
  marker.color.a = (doShow ? 1.0 : 0.0);
  pubMarker.publish(marker);
}

//================================= setMarker() ====================
void CDeliverySpec::setMarker(double posX, double posY, double rotZ) {
  marker.pose.position.x = posX;
  marker.pose.position.y = posY;
  marker.pose.orientation.z = rotZ;
  marker.pose.orientation.w = sqrt(1.0 - rotZ * rotZ);
  pubMarker.publish(marker);
}

void CDeliverySpec::setMarker(geometry_msgs::Pose pose) {
  marker.pose.position.x = pose.position.x;
  marker.pose.position.y = pose.position.y;
  // Z handled seperately
  marker.pose.orientation.z = pose.orientation.x;
  marker.pose.orientation.w = pose.orientation.y;
  marker.pose.orientation.z = pose.orientation.z;
  marker.pose.orientation.w = pose.orientation.w;
  pubMarker.publish(marker);
}

void CDeliverySpec::setMarkerZ(double posZ) {
  marker.pose.position.z = posZ;
  pubMarker.publish(marker);
}

void CDeliverySpec::moveMarkerZ(double posZ0, double posZ1) {
  int numPeriods = 50;
  float sleepPeriodSec = MARKER_LIFT_DURATION_SEC/numPeriods;

  double deltaZTotal = posZ1 - posZ0;
  double deltaZ = deltaZTotal/numPeriods;

  ros::Duration duration(sleepPeriodSec);
  double curZ = posZ0;
  float timeRemaining = MARKER_LIFT_DURATION_SEC;

  while (timeRemaining > 0){
    duration.sleep();
    timeRemaining -= sleepPeriodSec;
    curZ += deltaZ;
    setMarkerZ(curZ);
  }

  setMarkerZ(posZ1);
}

//================================= start() ====================
void CDeliverySpec::start() {
  setDeliveryState(eDeliveryState::GOTO_PICKUP);
}

//================================= setDeliveryState() ====================
void CDeliverySpec::setDeliveryState(eDeliveryState newState) {

  switch(newState){
    case eDeliveryState::IDLE:
      eState = newState;
      break;

    case eDeliveryState::GOTO_PICKUP:
      moveToGoal("GOTO_PICKUP", actionClientMoveBase, posePickup);
      eState = newState;
      break;

    case eDeliveryState::PICKINGUP:
      moveMarkerZ(MARKER_FLOOR_Z, MARKER_PICKUP_Z);
      eState = newState;
      setDeliveryState(eDeliveryState::GOTO_DROPPOFF);
      break;

    case eDeliveryState::GOTO_DROPPOFF:
      moveToGoal("GOTO_DROPPOFF", actionClientMoveBase, poseDropoff);
      eState = newState;
      break;

    case eDeliveryState::DROPPINGOFF:
      moveMarkerZ(MARKER_PICKUP_Z, MARKER_FLOOR_Z);;
      eState = newState;
      setDeliveryState(eDeliveryState::GOTO_HOME);
      break;

    case eDeliveryState::GOTO_HOME:
      eState = newState;
      moveToGoal("GOTO_HOME", actionClientMoveBase, poseRobotHome);
      break;

    default:
      break;
  }
}

//================================= cbOdom() ====================
void CDeliverySpec::cbOdom(const nav_msgs::Odometry::ConstPtr &msg) {
  geometry_msgs::Pose poseBot = msg->pose.pose;
  //ROS_INFO("IN cbOdom!!!<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  //ROS_INFO("In cbOdom: posBot= x=%g y=%g w=%g", poseBot.position.x, poseBot.position.y, poseBot.orientation.w);

  if (eState == eDeliveryState::GOTO_DROPPOFF) {
    setMarker(poseBot);
  }
}

//================================= cbGoalDone() ====================
void CDeliverySpec::cbGoalDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){
  //ROS_INFO("IN cbGoalDone!!!<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  eDeliveryState neweState;

  const char *strCurState = ToString(eState);

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("cbGoalDone(%s) SUCCEEDED", strCurState);

    switch(eState){
      case eDeliveryState::GOTO_PICKUP:
        neweState = eDeliveryState::PICKINGUP;
        break;

      case eDeliveryState::GOTO_DROPPOFF:
        neweState = eDeliveryState::DROPPINGOFF;
        break;

      case eDeliveryState::GOTO_HOME:
        neweState = eDeliveryState::IDLE;
        break;

      case eDeliveryState::DROPPINGOFF:
      case eDeliveryState::PICKINGUP:
      case eDeliveryState::IDLE:
      default:
        ROS_WARN("cbGoalDone(%s) called with unexpected state!", strCurState);
        neweState = eDeliveryState::IDLE;
        break;

    }

  } else {
    ROS_WARN("cbGoalDone(%s) FAILED", strCurState);
    neweState = eDeliveryState::IDLE;
  }
  setDeliveryState(neweState);
}

//================================= moveToGoal() ====================
bool CDeliverySpec::moveToGoal(const char *strGoal, MoveBaseClient &baseClient, geometry_msgs::Pose const &poseGoal) {

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = poseGoal;

  baseClient.sendGoal(goal, boost::bind(&CDeliverySpec::cbGoalDone, this, _1, _2));
}

//=============================================================
//================================= Utilities =============

//================================= createMarker() ====================
visualization_msgs::Marker createMarker() {
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "marker";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.
  marker.action = visualization_msgs::Marker::ADD;

  float markerSize = MARKER_SIZE;
  marker.scale.x = markerSize;
  marker.scale.y = markerSize;
  marker.scale.z = markerSize;

  // Set the pose of the marker.
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = markerSize/2.0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  return marker;
}

//================================= createPose() ====================
const geometry_msgs::Pose createPose(float px, float py, float pz, float qx, float qy, float qz, float qw) {
  geometry_msgs::Pose pose;
  pose.position.x = px;
  pose.position.y = py;
  pose.position.z = pz;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;

  return pose;
}
