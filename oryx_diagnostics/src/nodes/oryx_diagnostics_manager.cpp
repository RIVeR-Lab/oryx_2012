/**
 * @file	oryx_diagnostics_manager.cpp
 * @date	Dec 11, 2012
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */
#include<ros/ros.h>
#include<oryx_msgs/DiagnositicsRegistration.h>

#include"NodeManager.h"

using namespace oryx_diagnostics;

class DiagnosticsManager{
public:
	DiagnosticsManager(const std::string& name, ros::NodeHandle& nh):
	_name(name),
	_nodes(),
	_nh(nh){
		ROS_INFO_STREAM("Setting Up Oryx Diagnostics Manager <"<<name<<">...");
		this->_reg_srv = this->_nh.advertiseService("oryx/diagnostics/registration", &DiagnosticsManager::regCB, this);
		this->_nodes.registerNode(name,"diagnostics_manager",-1,0,0);
	}
	virtual ~DiagnosticsManager(){};

	void printList(){
		std::stringstream output;
		output<<"This is the Node List for Diagnostics Manager <"<<this->_name<<">\n"<<this->_nodes;
		ROS_INFO_STREAM(output.str());
	}

private:
	std::string			_name;
	NodeManager			_nodes;
	ros::NodeHandle		_nh;
	ros::ServiceServer	_reg_srv;

	bool regCB(	oryx_msgs::DiagnositicsRegistration::Request& req,
				oryx_msgs::DiagnositicsRegistration::Response& res){
		try{
			res.node_id = this->_nodes.registerNode(req.node_name, req.node_type, req.criticality, req.heartbeat_frequency, req.heartbeat_tolerence);
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
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_diagnostics_manager");
	ros::NodeHandle nh;

	nh.setParam("oryx/diagnostics", true);

	DiagnosticsManager manager(ros::this_node::getName(), nh);
	manager.printList();
	ros::spin();

}



