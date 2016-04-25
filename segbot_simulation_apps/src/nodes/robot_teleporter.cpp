#include <ros/ros.h>

#include <bwi_msgs/RobotTeleporterInterface.h>

bool execute(bwi_msgs::RobotTeleporterInterface::Request  &req,
             bwi_msgs::RobotTeleporterInterface::Response &res) {

  res.status = "";
  if (!req.room.empty() && !req.door.empty()) {


  if (req.all_doors) {
    if (req.open) {
      gh_->openAllDoors();
    } else {
      gh_->closeAllDoors();
    }
    res.success = true;
  } else {
    if (req.open) {
      res.success = gh_->openDoor(req.door);
    } else {
      res.success = gh_->closeDoor(req.door);
    }
    if (!res.success) {
      res.status = "Unable to resolved " + req.door + "!";
    }
  } else {
    res.success = False;
    res.status = "Malformed request. Either room(" + req.room + ") or door(" + req.door + ") is empty.";
  }

  return true;
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "gazebo_robot_teleporter");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("teleport_robot", execute);
  ros::spin();
  return 0;
}