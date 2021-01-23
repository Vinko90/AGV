#include <ros/ros.h>
#include <hector_exploration_planner/hector_exploration_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

class SimpleExplorationPlanner
{
public:
  SimpleExplorationPlanner()
  {
    ros::NodeHandle nh;

    costmap_2d_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tfl_);

    planner_ = new hector_exploration_planner::HectorExplorationPlanner();
    planner_->initialize("hector_exploration_planner",costmap_2d_ros_);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> goal_action_client_("/move_base", true);
    
    while(!goal_action_client_.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    while(ros::ok()){
      tf::Stamped<tf::Pose> robot_pose_tf;
      costmap_2d_ros_->getRobotPose(robot_pose_tf);

      geometry_msgs::PoseStamped pose;
      nav_msgs::Path path;
      
      move_base_msgs::MoveBaseGoal goal;
      tf::poseStampedTFToMsg(robot_pose_tf, pose);
      planner_->doExploration(pose, path.poses);
      path.header.frame_id = "map";
      path.header.stamp = ros::Time::now();

      goal.target_pose = path.poses[path.poses.size()-1];
      goal_action_client_.sendGoal(goal);
      
//       while(goal_action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
// 	  actionlib::SimpleClientGoalState state = goal_action_client_.getState();
// 	  if(state != actionlib::SimpleClientGoalState::ACTIVE && state != actionlib::SimpleClientGoalState::PENDING){
// 	    if(state == actionlib::SimpleClientGoalState::LOST){
// 	     ROS_WARN("Robot lost."); 
// 	    }
// 	    if(state == actionlib::SimpleClientGoalState::PREEMPTED){
// 	     ROS_WARN("Robot preempted."); 
// 	    }
// 	    if(state == actionlib::SimpleClientGoalState::ABORTED){
// 	     ROS_WARN("Robot abroted."); 
// 	    }
// 	    if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
// 	      ROS_INFO("Robot reached goal."); 
// 	    }else{
// 	      ROS_WARN("WTF??");
// 	    }
// 	    
// 	    break;
// 	  }
// 	}
    }
  }

protected:
  hector_exploration_planner::HectorExplorationPlanner* planner_;
  ros::Publisher exploration_plan_pub_;
  costmap_2d::Costmap2DROS* costmap_2d_ros_;
  tf::TransformListener tfl_;

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "agv_hector_exploration");

  SimpleExplorationPlanner ep;

  ros::spin();

  return 0;
}
