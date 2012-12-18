/**
 * @file	oryx_diagnostics_manager.cpp
 * @date	Dec 11, 2012
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */
#include<ros/ros.h>
#include<oryx_msgs/DiagnositicsRegistration.h>
#include<oryx_diagnostics/DiagnosticsCommand.h>
#include<boost/unordered/unordered_map.hpp>

#include"NodeManager.h"

using namespace oryx_diagnostics;

class DiagnosticsManager{
public:
	DiagnosticsManager(const std::string& name, ros::NodeHandle& nh):
	name_(name),
	nodes_(),
	nh_(nh){
		ROS_INFO_STREAM("Setting Up Oryx Diagnostics Manager <"<<name<<">...");
		this->reg_srv_ = this->nh_.advertiseService("oryx/diagnostics/registration", &DiagnosticsManager::regCB, this);
		this->com_srv_ = this->nh_.advertiseService("oryx/diagnostics/commands", &DiagnosticsManager::comCB, this);
		this->nodes_.registerNode(name,"diagnostics_manager",-1,0,0);

	}
	virtual ~DiagnosticsManager(){};

	void printList(){
		std::stringstream output;
		output<<"This is the Node List for Diagnostics Manager <"<<this->name_<<">\n"<<this->nodes_;
		ROS_INFO_STREAM(output.str());
	}

private:
	std::string			name_;
	NodeManager			nodes_;
	ros::NodeHandle		nh_;
	ros::ServiceServer	reg_srv_;
	ros::ServiceServer  com_srv_;

	typedef boost::function<bool (std::string& data, oryx_diagnostics::DiagnosticsCommand::Response& response)> command_func_;	///Typedef defining a boost::function object wrapper specifying the callback signature of commands
	boost::unordered_map<std::string, command_func_> commands_;			///boost::unordered_map mapping the string command name with its function

	bool regCB(	oryx_msgs::DiagnositicsRegistration::Request& req,
				oryx_msgs::DiagnositicsRegistration::Response& res){
		try{
			res.node_id = this->nodes_.registerNode(req.node_name, req.node_type, req.criticality, req.heartbeat_frequency, req.heartbeat_tolerence);
			res.success = true;
			return true;
		}catch(oryx_diagnostics::NodeManagerException& e){
			res.success = false;
			return false;
		}catch(std::exception& e){
			ROS_ERROR("%s", e.what());
		}
		return false;
	}

	bool comCB( oryx_diagnostics::DiagnosticsCommand::Request& req,
				oryx_diagnostics::DiagnosticsCommand::Response& response){
		if(this->commands_.count(req.command)!=1){
			response.success = false;
			response.data = "Invalid Command:" + req.command;
			return true;
		}else{
			return this->commands_[req.command](req.data, response);
		}
		return false;
	}

	bool listCB(std::string& data, oryx_diagnostics::DiagnosticsCommand::Response& response){
		std::stringstream res_data;
		this->nodes_.printList(res_data);
		response.data = res_data.str();
		return true;
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_diagnostics_manager");
	ros::NodeHandle nh;

	nh.setParam("oryx/diagnostics", true);

	DiagnosticsManager manager(ros::this_node::getName(), nh);
	manager.printList();
	ros::spin();

}



