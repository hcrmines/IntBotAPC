#include <stdlib.h>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include <amazon/MoveToAction.h>
#include <actionlib/client/simple_action_client.h>

class MoveController{
	public:
		MoveController();
		void pickObject();
	protected:
		actionlib::SimpleActionClient<amazon::MoveToAction> client;
		ros::NodeHandle nh;
};

MoveController::MoveController() : client("MoveTo", true) {
	client.waitForServer();
}

void MoveController::pickObject(){

	amazon::MoveToGoal goal;
	goal.moveAction = amazon::MoveToGoal::MOVE_TO_PICK;
	goal.movePose.position.x = 1.10;
	goal.movePose.position.y = 0.35;
	goal.movePose.position.z = 1.65;
	goal.movePose.orientation.w = 1.0;
	std_msgs::String shelf;
	while(nh.ok()){
		std::cout << "Enter shelf name (single character): ";
		std::getline(std::cin, shelf.data);
		goal.shelf = shelf;
		
		client.sendGoal(goal);
		client.waitForResult(ros::Duration(60.0));
		if(client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_ERROR("Could not get to requested point");
		}
	}
}

int main(int argc, char **argv){
	ROS_INFO("Initializing test_move_client");
	ros::init(argc, argv, "test_move_client");
	MoveController controller;
	controller.pickObject();

	return 0;
}
