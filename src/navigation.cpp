#include <navigation.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <ros/service_client.h>
#include <nav_msgs/GetPlan.h>
#include <ros/node_handle.h>

bool Navigation::moveToGoal(float xGoal, float yGoal, float phiGoal){
	// Set up and wait for actionClient.
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
	// Set goal.
    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x =  xGoal;
    goal.target_pose.pose.position.y =  yGoal;
    goal.target_pose.pose.position.z =  0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = phi.z;
    goal.target_pose.pose.orientation.w = phi.w;
    ROS_INFO("Sending goal location ...");
	// Send goal and wait for response.
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("You have reached the destination");
        return true;
    } else {
        ROS_INFO("The robot failed to reach the destination");
        return false;
    }
}

bool Navigation::checkPlan(ros::NodeHandle& nh, float xStart, float yStart, float phiStart, float xGoal, float yGoal, float phiGoal){
	// Set up and wait for actionClient.
    bool callExecuted, validPlan;
/*     actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    } */
	
    //Set start position
    geometry_msgs::PoseStamped start;
    geometry_msgs::Quaternion phi1 = tf::createQuaternionMsgFromYaw(phiStart);
    start.header.seq = 0;
    start.header.stamp = ros::Time::now();
    start.header.frame_id = "map";
    start.pose.position.x = xStart;
    start.pose.position.y = yStart;
    start.pose.position.z = 0.0;
    start.pose.orientation.x = 0.0;
    start.pose.orientation.y = 0.0;
    start.pose.orientation.z = phi1.z;
    start.pose.orientation.w = phi1.w;

    //Set goal position
    geometry_msgs::PoseStamped goal;
    geometry_msgs::Quaternion phi2 = tf::createQuaternionMsgFromYaw(phiGoal);
    goal.header.seq = 0;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = xGoal;
    goal.pose.position.y = yGoal;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = phi2.z;
    goal.pose.orientation.w = phi2.w;
    
    ros::ServiceClient check_path = nh.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
  
    callExecuted = check_path.call(srv);
    if (callExecuted){
        ROS_INFO("Call to check plan sent");
    }
    else{
       ROS_INFO("Call to check plan NOT sent"); 
    }
    
    ROS_INFO("Plan size: %ld", srv.response.plan.poses.size());
    if(srv.response.plan.poses.size() > 0){
        validPlan = true;
        ROS_INFO("Successful plan");
    }
    else{
        validPlan = false;
        ROS_INFO("Unsuccessful plan");
    }
    return validPlan;
}