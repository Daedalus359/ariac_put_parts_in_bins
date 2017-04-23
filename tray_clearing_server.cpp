#include <ros/ros.h>
#include <cwru_ariac/Part.h>
#include <cwru_ariac/InventoryServiceMsg.h>
#include <cwru_ariac/TrayClearServiceMsg.h>
#include "CameraEstimator.h"
#include <iostream>
#include <string>
#include <RobotMove.h>

//------------------------------node summary-------------------------------------------

//the overall functionality of this node will be to accept a list of part names and a list of corresponding part poses (which ought to be of equal length) these parts are to be taken from their current poses to available space in the bins, with the intended purpose being the partial or full clearning of parts occupying an AGV.

//this code interfaces with code written by Bruce Liu. Bruce's inventory_control_service node takes in a name for a part and returns an array geometry_msgs/PoseStamped called poses

//the name for the part will correspond to a lookup about that part's radius, so then name provides information about the type of part

//one request will be made to Bruce's node for the first part from the list of inputs. The first pose from the returned list of posestampeds will be selected, and a request will be made to move the part from the current pose to the second one. Then the first part will be removed from the list of inputs, and the process will repeat until one of the input arrays is empty (for a well formed input, this will happen to both input arrays at once)

//--------------------------------------------------------------------------------------

ros::ServiceClient storageFinderClient;
cwru_ariac::InventoryServiceMsg storageFinderMsg;
cwru_ariac::TrayClearServiceMsg trayClearMsg;
RobotMove robotMove;
//TODO: I don't think the above line does it, consult Shipei

bool clearTrayCallback(cwru_ariac::TrayClearServiceMsgRequest& request, cwru_ariac::TrayClearServiceMsgResponse& response){
	ROS_INFO("tray clear request received");
	
	//get the data from the request in a more useful form
	std::vector<string> part_names_vec(request.part_names, request.part_names + sizeof request.part_names / sizeof request.part_names[0]);
	
	std::vector<geometry_msgs/PoseStamped> initial_poses_vec(request.initial_poses, request.initial_poses + sizeof request.initial_poses / sizeof request.initial_poses[0]);
	
	//declarations to be used in the following while loop
	string next_part_name;
	geometry_msgs/PoseStamped next_initial_location;
	geometry_msgs/PoseStamped next_final_destination;
	Part pick_part, place_part;//for populating a move command
	
	while ((!part_names_vec.empty())&&(!initial_poses_vec.empty())){
		//take out the next part and desired pose, then remove from the list
		next_part_name = part_names_vec[part_names_vec.size - 1];
		next_initial_location = initial_poses_vec[initial_poses_vec.size - 1];
		part_names_vec.pop_back;
		initial_poses_vec.pop_back;

		//get the destination pose for this part from Bruce
		storageFinderMsg.request.part_name = next_part_name;
		
		if(client.call(storageFinderMsg)){
			//executes if a location was found
			next_final_destination = storageFinderMsg.response.poses[0];
			
			//populate a part data type with the info needed
			pick_part.name = next_part_name;
			place_part.name = next_part_name;
			pick_part.pose = next_initial_location;
			place_part.pose = next_final_destination;
			

			//TODO: ask shipei how to populate the location portion or why it's needed
			pick_part.location = Part::AGV1;
			place_part.location = Part::Bin6;

			//order the motion
			robotMove.enableAsync();//TODO: is this needed?
			robotMove.move(pick_part, place_part, 10.0);
			
			//wait for the motion to finish
			while(!robotMove.actionFinished()){
				ROS_INFO("Waiting for result");
				ros::Duration(1).sleep();
			}
			
		}else{
			ROS_WARN("failed to find location for a part");
		}
		
	}
}

int main(int argc, char **argv){
	
	//initialize node
	ros::init(argc, argv, "agv_clearing_server");
	ros::NodeHandle n;
	
	//set up communications with the location finder node
	storageFinderClient = n.serviceClient<cwru_ariac::InventoryServiceMsg>("look_up_parts_space");
	while (!client.exists()) {
			ROS_WARN("waiting for location finder service...");
			ros::Duration(1.0).sleep();
    }
	ROS_INFO("connected to location finder service");
	
	//set up a server to take requests of parts to clear
	ros::ServiceServer service = nh.advertiseService("clear_avg_tray",clearTrayCallback);
	ROS_INFO("ready to accept tray clearing commands");
	
	ros::spin();
	return 0;
}
