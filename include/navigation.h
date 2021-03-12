#pragma once
#include <ros/ros.h>

class Navigation {
	public:
		static bool moveToGoal(float xGoal, float yGoal, float phiGoal);
};