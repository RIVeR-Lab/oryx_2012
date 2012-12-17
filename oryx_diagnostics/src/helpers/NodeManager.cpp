/**
 * @file	NodeManager.cpp
 * @date	Dec 11, 2012
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */

#include "NodeManager.h"
#include <boost/foreach.hpp>

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

const NodeManager::container& NodeManager::getNode(int node_id) const throw(NoMatchingNodeId){
	checkId(node_id);
	return this->_nodes.at(node_id);
}

void NodeManager::touch(int node_id)throw(NoMatchingNodeId){
	checkId(node_id);
	this->_nodes[node_id].touch();
}

void NodeManager::deleteNode(int node_id)throw(NoMatchingNodeId){
	checkId(node_id);
	this->_nodes.erase(node_id);
}

bool NodeManager::checkId(int node_id)const throw(NoMatchingNodeId){
	if(this->_nodes.count(node_id)==1){
		return true;
	}else throw NoMatchingNodeId(node_id);
	return false;
}

void NodeManager::printList(std::stringstream& output) const{
	BOOST_FOREACH(node_map::value_type item, this->_nodes){
		output<<"["<<item.first<<"]"<<item.second<<"\n";
	}
}

const NodeManager::container& NodeManager::operator[](int node_id) const throw(NoMatchingNodeId){
	return getNode(node_id);
}

std::ostream& operator<<(std::ostream& out, const NodeManager& rhs){
	BOOST_FOREACH(NodeManager::node_map::value_type item, rhs._nodes){
		out<<"["<<item.first<<"]"<<item.second<<"\n";
	}
	return out;
}

} /* namespace oryx_diagnostics */
