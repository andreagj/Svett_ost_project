#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ascend_msgs/AIInterfaceAction.h>

//Typedefs
using ActionServerType = actionlib::SimpleActionServer<ascend_msgs::AIInterfaceAction>;
using GoalType = ascend_msgs::AIInterfaceGoal;


mavros_msgs::PositionTarget setpoint;
geometry_msgs::PoseStamped mocap_pos;


constexpr uint16_t MOVE_TYPE = 2552;
constexpr uint16_t LAND_TYPE = 10744;
constexpr uint16_t TAKEOFF_TYPE = 6648;
constexpr uint16_t IDLE_TYPE = 18936;

int state; 


void newGoalCB(ActionServerType* server){
	if(!server->isNewGoalAvailable()) return;
	//Accept the new goal
	auto goal = server->acceptNewGoal();//Returns boost::shared_ptr to goal
	//Check that the client hasnt cancelled the request already calls
	if(server->isPreemptRequested()) {
	//Goal is already stopped by client
	return;
	}
	//Read the goal and to somehting about it


	state = goal.type

	setpoint.position.x = goal.x; 
	setpoint.position.y = goal.y;
	setpoint.position.z = goal.z; 
	setpoint.yaw = -PI/2.0; 

}

void preemptCB(ActionServerType* server){

	ROS_WARN("Preempted!");
}

void mocapCallback(const geometry_msgs::PoseStamped& input){
    mocap_pos = input;
}


bool isGoalCompleted(){
	if( -0.2 + mocap_pos.position.x > setpoint.position.x || 0.2 + mocap_pos.position.x < setpoint.position.x
		&& -0.2 + mocap_pos.position.y > setpoint.position.y || 0.2 + mocap_pos.position.y < setpoint.position.y
		&& -0.2 + mocap_pos.position.z > setpoint.position.z || 0.2 + mocap_pos.position.z < setpoint.position.z){
		return true;
	}else{
		return false; 
	}

}


int main(int argc, char** argv){	
	ros::init(argc,argv	, "control"); 
	ros::NodeHandle n;
	ActionServerType server(n, "control_action_server", false);

	state = 3; // Idle 

	server.registerGoalCallback(boost::bind(newGoalCB, &server));
	server.registerPreemptCallback(boost::bind(preemptCB, &server));
	ros::Rate rate(30.0);
	server.start();

	ros::Subscriber mocap_sub = n.subscribe("mavros/mocap/pose", 10, mocapCallback);
	//ros::Subscriber landed_sub = n.subscribe()
	pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);	



	while(ros::ok()){

		ros::spinOnce();c

		switch state{
			case 0: // Land
				setpoint.type_mask = LAND_TYPE; 
				break;
			case 1: // Takeoff
				setpoint.type_mask = TAKEOFF_TYPE; 
				break; 
			case 2: // Move 
				setpoint.type_mask = MOVE_TYPE;
				break;
			case 3: // Idle
				setpoint.type_mask = IDLE_TYPE;
				break 
			default: 
				break; 
		}
		pub.publish(setpoint);


		if(isGoalCompleted() && server.isActive()) {
			ascend_msgs::AIInterfaceResult result;
			result = true; 
			server.setSucceeded(result);
		}

		rate.sleep(); 
	}
}

