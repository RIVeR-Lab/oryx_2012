/**
 * @file	oryx_diagnostics_client.cpp
 * @date	Dec 18, 2012
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */

#include <ros/ros.h>
#include <oryx_diagnostics/DiagnosticsCommand.h>


int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_diagnostics_client");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	ros::ServiceClient client = nh.serviceClient<oryx_diagnostics::DiagnosticsCommand>("oryx/diagnostics/commands");

	std::string command;
	std::string arguments;

	pnh.getParam("command", command);
	pnh.getParam("arg", arguments);

	if(command == "") ROS_WARN("No Command Given!");
	else{
		oryx_diagnostics::DiagnosticsCommand msg;
		msg.request.command = command;
		msg.request.data = arguments;
		if(client.call(msg.request, msg.response)){
			if(msg.response.success)
			{
				ROS_INFO("%s", msg.response.data.c_str());
			}
			else
			{
				ROS_WARN("Error With Command: %s", msg.response.data.c_str());
			}

		}
		else
		{
			ROS_ERROR("Could Not Communicate With Server!");
		}
	}
}

