#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <actionlib/client/simple_action_client.h>

#define MARKER_LIFT_DURATION_SEC (5.0f)
#define MARKER_SIZE 0.2
#define ROBOT_HEIGHT 0.4
#define MARKER_FLOOR_Z (MARKER_SIZE/2)
#define MARKER_PICKUP_Z (ROBOT_HEIGHT + (MARKER_SIZE/2))

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum class eDeliveryState { IDLE, GOTO_PICKUP, PICKINGUP, GOTO_DROPPOFF, DROPPINGOFF, GOTO_HOME };
inline const char* ToString(eDeliveryState eState)
{
  switch (eState)
  {
    case eDeliveryState::IDLE:   return "IDLE";
    case eDeliveryState::GOTO_PICKUP:   return "GOTO_PICKUP";
    case eDeliveryState::PICKINGUP: return "PICKINGUP";
    case eDeliveryState::GOTO_DROPPOFF: return "GOTO_DROPPOFF";
    case eDeliveryState::DROPPINGOFF: return "DROPPINGOFF";
    case eDeliveryState::GOTO_HOME: return "GOTO_HOME";
    default: return "[Unknown eDeliveryState]";
  }
}
class CDeliverySpec {
public:
    geometry_msgs::Pose posePickup;
    geometry_msgs::Pose poseDropoff;
    ros::Publisher pubMarker;
    visualization_msgs::Marker marker;

    //CDeliverySpec(ros::Publisher pubMarker, MoveBaseClient actionClientMoveBase);
    CDeliverySpec();
    void init();

    void setDeliveryState(eDeliveryState newState);
    void start();
    void addDelivery(geometry_msgs::Pose posePickup, geometry_msgs::Pose poseDropoff);
    void setMarker(double posX, double posY, double rotZ);
    void setMarker(geometry_msgs::Pose pose);
    void cbGoalDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    bool moveToGoal(const char *strGoal, MoveBaseClient &baseClient, geometry_msgs::Pose const &poseGoal);

private:
    eDeliveryState eState;
    geometry_msgs::Pose poseRobotHome;
    MoveBaseClient actionClientMoveBase;
    ros::Subscriber subOdom;

    void cbOdom(const nav_msgs::Odometry::ConstPtr &msg);
    void setMarkerZ(double posZ);
    void moveMarkerZ(double posZ0, double posZ1);
    void showMarker(bool doShow);

};


const geometry_msgs::Pose createPose(float px, float py, float pz = 0.5, float qx = 0, float qy = 0, float qz = 0, float qw = 1.0);
visualization_msgs::Marker createMarker();
