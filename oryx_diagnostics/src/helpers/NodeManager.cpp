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
	this->next_id_ = 0;
}

NodeManager::~NodeManager() {
	// TODO Auto-generated destructor stub
}

int NodeManager::registerNode(const std::string& node_name, const std::string& node_type, int criticality, int heartbeat_rate, int heartbeat_tolerence)throw(NonUniqueNodeName){
	container _n(node_name, node_type, criticality, heartbeat_rate, heartbeat_tolerence);
	this->nodes_[this->next_id_]=_n;
	this->next_id_++;
	return this->next_id_-1;
}

const NodeManager::container& NodeManager::getNode(int node_id) const throw(NoMatchingNodeId){
	checkId(node_id);
	return this->nodes_.at(node_id);
}

void NodeManager::touch(int node_id)throw(NoMatchingNodeId){
	checkId(node_id);
	this->nodes_[node_id].touch();
}

void NodeManager::deleteNode(int node_id)throw(NoMatchingNodeId){
	checkId(node_id);
	this->nodes_.erase(node_id);
}

bool NodeManager::checkId(int node_id)const throw(NoMatchingNodeId){
	if(this->nodes_.count(node_id)==1){
		return true;
	}else throw NoMatchingNodeId(node_id);
	return false;
}

void NodeManager::printList(std::stringstream& output) const{
	BOOST_FOREACH(node_map::value_type item, this->nodes_){
		output<<"["<<item.first<<"]"<<item.second<<"\n";
	}
}

void NodeManager::timeCheck(result_map& results, ros::Time& checkTime) const{
	BOOST_FOREACH(node_map::value_type item, this->nodes_){
		//If the heartbeat rate check in enabled by being greater than 0
		if(item.second.getHeartbeatRate()>0){
			//Calculate all the durations
			ros::Duration tolerence(item.second.getHeartbeatTolerence());
			ros::Duration check_rate(item.second.getHeartbeatRate());
			ros::Duration heart_rate(checkTime-item.second.getLastTouch());

			results[item.first].second = item.second.getCriticality();
			//If the duration of the last heartbeat was longer than the expected duration plus tolerence, it failed
			if(heart_rate>(check_rate+tolerence)){
				results[item.first].first = false;
			}
			//Otherwise it passed
			else{
				results[item.first].first = true;
			}
		}
		//Otherwise it automatically passes
		else{
			results[item.first].second = item.second.getCriticality();
			results[item.first].first  = true;
		}
	}
}

const NodeManager::container& NodeManager::operator[](int node_id) const throw(NoMatchingNodeId){
	return getNode(node_id);
}

std::ostream& operator<<(std::ostream& out, const NodeManager& rhs){
	BOOST_FOREACH(NodeManager::node_map::value_type item, rhs.nodes_){
		out<<"["<<item.first<<"]"<<item.second<<"\n";
	}
	return out;
}

} /* namespace oryx_diagnostics */
