/**
 * @file	DiagnosticsManager.h
 * @date	Jan 23, 2013
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */

#ifndef DIAGNOSTICSMANAGER_H_
#define DIAGNOSTICSMANAGER_H_
//************************** SYSTEM DEPENDANCIES ********************************//
#include<ros/ros.h>
#include<oryx_diagnostics/DiagnosticsRegistration.h>
#include<oryx_msgs/Heartbeat.h>
#include<oryx_msgs/SoftwareStop.h>
#include<oryx_diagnostics/DiagnosticsCommand.h>
#include<boost/unordered/unordered_map.hpp>
#include<boost/foreach.hpp>
#include<boost/function.hpp>
#include<boost/bind.hpp>

//************************** LOCAL DEPENDANCIES ********************************//
#include"NodeManager.h"

//************************** NAMESPACES ***************************************//
using std::string;
using std::stringstream;
using boost::function;
using boost::unordered_map;

namespace oryx_diagnostics{
class DiagnosticsManager{
private:
	typedef function<bool (string& data, oryx_diagnostics::DiagnosticsCommand::Response& response)> command_func_;	///Typedef defining a function object wrapper specifying the callback signature of commands
	typedef unordered_map<string, command_func_> command_map;
public:
	DiagnosticsManager(const string& name, ros::NodeHandle& nh);
	virtual ~DiagnosticsManager(){};

	void printList();

	void runDiagnostics();

private:
	string			name_;
	NodeManager			nodes_;
	ros::NodeHandle		nh_;
	ros::ServiceServer	reg_srv_;
	ros::ServiceServer  com_srv_;
	ros::Subscriber		hb_sub_;
	ros::Publisher		soft_stop_pub_;
	ros::Duration		max_skew_;


	command_map commands_;			///unordered_map mapping the string command name with its function

	/**
	 * Callback for processing requests to register new nodes with oryx_diagnostics
	 * @param req The service request
	 * @param res The service response
	 * @return TRUE if sucessful, else false
	 */
	bool regCB(	oryx_diagnostics::DiagnosticsRegistration::Request& req,
			oryx_diagnostics::DiagnosticsRegistration::Response& res);

	/**
	 * Callback for handling receiving heartbeat messages from nodes
	 */
	void hbCB(const oryx_msgs::HeartbeatConstPtr& msg);


	/**
	 * Callback for servicing DiagnosticsCommand services
	 * @param req The service request
	 * @param response The service response
	 * @return TRUE if sucessful, else false
	 */
	bool comCB( oryx_diagnostics::DiagnosticsCommand::Request& req,
			oryx_diagnostics::DiagnosticsCommand::Response& response);

	/**
	 * Callback for handling the "node_list" command. Fills the response.data field with the same string as would result
	 * from calling DiagnosticsManager::printList
	 * @param data The callback argument data (in this case, ignored)
	 * @param response The DiagnosticsCommand::Response reference to fill with data
	 * @return TRUE if sucessful, else FALSE
	 */
	bool listCB(string& data, oryx_diagnostics::DiagnosticsCommand::Response& response);
};
};


#endif /* DIAGNOSTICSMANAGER_H_ */
