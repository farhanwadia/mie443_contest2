#pragma once
#include <ros/ros.h>

class Navigation {
	public:
		static bool moveToGoal(float xGoal, float yGoal, float phiGoal);
		static bool checkPlan(ros::NodeHandle& nh, float xStart, float yStart, float phiStart, float xGoal, float yGoal, float phiGoal);
};
