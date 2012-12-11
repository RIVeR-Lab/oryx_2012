/**
 * @file	NodeManager.cpp
 * @date	Dec 11, 2012
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */

#include "NodeManager.h"

using namespace oryx_diagnostics;
namespace oryx_diagnostics {

NodeManager::NodeManager() {
	this->_next_id = 0;
}

NodeManager::~NodeManager() {
	// TODO Auto-generated destructor stub
}

int NodeManager::registerNode(const std::string& node_name, const std::string& node_type, int criticality, int heartbeat_rate, int heartbeat_tolerence)throw(NonUniqueNodeName){
	container _n(node_name, node_type, criticality, heartbeat_rate, heartbeat_tolerence);
	this->_nodes[this->_next_id]=_n;
	this->_next_id++;
	return this->_next_id-1;
}

const NodeManager::container& NodeManager::getNode(int node_id)throw (NoMatchingNodeId)const{
	if(this->_nodes.count(node_id)!=1) throw NoMatchingNodeId(node_id);
}

} /* namespace oryx_diagnostics */
