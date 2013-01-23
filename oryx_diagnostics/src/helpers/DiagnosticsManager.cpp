/**
 * @file	DiagnosticsManager.cpp
 * @date	Jan 23, 2013
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */

#include"DiagnosticsManager.h"

namespace oryx_diagnostics{

DiagnosticsManager::DiagnosticsManager(const string& name, ros::NodeHandle& nh):
						name_(name),
						nodes_(),
						nh_(nh),
						max_skew_(1.0/250.0){
	//Printout startup information
	ROS_INFO_STREAM("Setting Up Oryx Diagnostics Manager <"<<name<<">...");

	//Register services/subscribers with ROS
	this->reg_srv_ = this->nh_.advertiseService("oryx/diagnostics/registration", &DiagnosticsManager::regCB, this);
	this->com_srv_ = this->nh_.advertiseService("oryx/diagnostics/commands", &DiagnosticsManager::comCB, this);
	this->hb_sub_  = this->nh_.subscribe("oryx/diagnostics/heartbeat", 10, &DiagnosticsManager::hbCB, this);
	this->soft_stop_pub_ = this->nh_.advertise<oryx_msgs::SoftwareStop>("oryx/software_stop", 2);

	//Register itself with its node manager
	this->nodes_.registerNode(name,"diagnostics_manager",0,-1,0);
	//Register commands
	this->registerCommands();

}
DiagnosticsManager::~DiagnosticsManager(){};

void DiagnosticsManager::registerCommands(){
	//Register valid commands for communication with oryx_diagnostics_client
	this->commands_["node_list"]=boost::bind(&DiagnosticsManager::listCB, this, _1, _2);

	//Print out the supported command list
	ROS_INFO("Registered Commands...");
	BOOST_FOREACH(command_map::value_type item, this->commands_){
		ROS_INFO("Registered Command: %s", item.first.c_str());
	}
}

void DiagnosticsManager::printList(){
	stringstream output;
	output<<"This is the Node List for Diagnostics Manager <"<<this->name_<<">\n"<<this->nodes_;
	ROS_INFO_STREAM(output.str());
}

void DiagnosticsManager::runDiagnostics(){
	while(ros::ok()){
		NodeManager::result_map results;
		//Get the current time to check against for heartbeat failure
		ros::Time check_time(ros::Time::now());
		//Get the list results of heartbeat check
		this->nodes_.timeCheck(results ,check_time);
		//For each item in the results map...
		BOOST_FOREACH(NodeManager::result_map::value_type item, results){
			bool failure  = false;
			bool critical = false;
			try{
				//If it failed...
				if(!item.second.first){
					//If it's critical, immediately send out the SoftwareStop message
					if(item.second.second==0){
						oryx_msgs::SoftwareStop msg;
						msg.message = "Critical Node Heartbeat Failure";
						msg.stop	= true;
						msg.header.stamp = ros::Time::now();
						msg.header.frame_id = "/robot";
						this->soft_stop_pub_.publish(msg);
						critical = true;
					}
					failure = true;
				}
			}catch(std::exception& e){
				ROS_ERROR("Unexpected Error While Checking Heartbeats: %s", e.what());
			}
			//If there was a failure, print out some info
			if(failure||critical){
				NodeManager::NodeInformationContainer info = this->nodes_.getNode(item.first);
				//If it was a critical failure, log to ROS_ERROR
				if(failure&&critical){
					ROS_ERROR_STREAM("Critical Node "<<info.getName()<<" Has Failed its Heartbeat Check, Soft-Stop Sent!!!!");
				}
				//Otherwise log to ROS_WARN
				else{
					ROS_WARN_STREAM("Non-Critical Node "<<info.getName()<<" Has Failed its Heartbeat Check!!!!");
				}
			}
		}
		ros::spinOnce();
	}
}

/**
 * Callback for processing requests to register new nodes with oryx_diagnostics
 * @param req The service request
 * @param res The service response
 * @return TRUE if sucessful, else false
 */
bool DiagnosticsManager::regCB(	oryx_diagnostics::DiagnosticsRegistration::Request& req,
		oryx_diagnostics::DiagnosticsRegistration::Response& res){
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

/**
 * Callback for handling receiving heartbeat messages from nodes
 */
void DiagnosticsManager::hbCB(const oryx_msgs::HeartbeatConstPtr& msg){
	ros::Duration skew((msg->timestamp-ros::Time::now()));
	if(skew>this->max_skew_){
		ROS_ERROR_STREAM("Warning, received a heartbeat from <"<<msg->node_id<<"> With a skew of "<<skew<<", greater than max skew of "<<this->max_skew_);
	}
	try{
		this->nodes_.touch(msg->node_id);
	}catch(NoMatchingNodeId& e){
		ROS_ERROR("%s", e.what());
	}catch(std::exception& e){
		ROS_ERROR("Unexpected Error: %s", e.what());
	}
}


/**
 * Callback for servicing DiagnosticsCommand services
 * @param req The service request
 * @param response The service response
 * @return TRUE if sucessful, else false
 */
bool DiagnosticsManager::comCB( oryx_diagnostics::DiagnosticsCommand::Request& req,
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

/**
 * Callback for handling the "node_list" command. Fills the response.data field with the same string as would result
 * from calling DiagnosticsManager::printList
 * @param data The callback argument data (in this case, ignored)
 * @param response The DiagnosticsCommand::Response reference to fill with data
 * @return TRUE if sucessful, else FALSE
 */
bool DiagnosticsManager::listCB(string& data, oryx_diagnostics::DiagnosticsCommand::Response& response){
	stringstream res_data;
	res_data<<"This is the current node list:\n";
	this->nodes_.printList(res_data);
	response.data = res_data.str();
	response.success = true;
	return true;
}
};
