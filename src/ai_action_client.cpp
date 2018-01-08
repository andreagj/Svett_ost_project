#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ascend_msgs/AIInterfaceAction.h>

using ClientType = actionlib::SimpleActionClient<ascend_msgs::AIInterfaceAction>;

struct point_t {
	float x;
	float y;
	float z;
};

void takeOff(ClientType& client) {
	ascend_msgs::AIInterfaceGoal goal;
	//Bruker konstantene definert i .action filen
	goal.type = ascend_msgs::AIInterfaceGoal::TAKEOFF;
	goal.z = 2.0; //Takeoff altitude

	//Send goal
	client.sendGoal(goal);
	//Wait for maximum 20 seconds
	bool finished_before_timeout = client.waitForResult(ros::Duration(20));
	//Check status
	if(finished_before_timeout) {
		auto state = client.getState();
		if(state == state.SUCCEEDED) {
			//Pointer to result
			auto result_p = client.getResult();
			
			ROS_INFO("Result: %i", result_p->complete);
			//yay it worked!
		} else if(state == state.ABORTED) {
			ROS_INFO("Something went wrong");
		}
	} else {
		client.cancelGoal();
	}
}

bool land(ClientType& client){
	ascend_msgs::AIInterfaceGoal goal;
	//Bruker konstantene definert i .action filen
	goal.type = ascend_msgs::AIInterfaceGoal::LAND;

	//Send goal
	client.sendGoal(goal);
	//Wait for maximum 20 seconds
	bool finished_before_timeout = client.waitForResult(ros::Duration(20));
	//Check status
	if(finished_before_timeout) {
		auto state = client.getState();
		if(state == state.SUCCEEDED) {
			//Pointer to result
			auto result_p = client.getResult();
			
			ROS_INFO("Result: %i", result_p->complete);
			//yay it worked!
			return true;
		} else if(state == state.ABORTED) {
			ROS_INFO("Something went wrong");
		}
	} else {
		client.cancelGoal();
	}
	return false;	
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "ai");
	ClientType client("control_action_server", true);
	client.waitForServer(); //Waits until server is ready

	std::vector<point_t> points;
	point_t point1 = {.x = 4.0, .y = 4.0, .z = 1.0};
	points.push_back(point1);
	point_t point2 = {.x = -4.0, .y = 4.0, .z = 1.0};
	points.push_back(point2);
	point_t point3 = {.x = 4.0, .y = -4.0, .z = 1.0};
	points.push_back(point3);
	point_t point4 = {.x = -4.0, .y = -4.0, .z = 1.0};
	points.push_back(point4);


	takeOff(client);
	
	ascend_msgs::AIInterfaceGoal goal;

	for (int i = 0; i < points.size(); i++) {
		goal.type = ascend_msgs::AIInterfaceGoal::MOVE;
		goal.x = points[i].x;
		goal.y = points[i].y;
		goal.z = 2.0;

		//Send goal
		client.sendGoal(goal);
		//Wait for maximum 20 seconds
		bool finished_before_timeout = client.waitForResult(ros::Duration(20));
		//Check status
		if(finished_before_timeout) {
			auto state = client.getState();
			if(state == state.SUCCEEDED) {
				//Pointer to result
				auto result_p = client.getResult();
				
				ROS_INFO("Result: %i", result_p->complete);

				if (land(client)){
					takeOff(client);
				}

				//yay it worked!
			} else if(state == state.ABORTED) {
				ROS_INFO("Something went wrong");
			}
		} else {
			client.cancelGoal();
		}
	}
}