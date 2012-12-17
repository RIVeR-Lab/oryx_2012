/**
 * @file	NodeManager.h
 * @date	Dec 11, 2012
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */

#ifndef NODEMANAGER_H_
#define NODEMANAGER_H_

#include<ros/ros.h>
#include<boost/unordered/unordered_map.hpp>

namespace oryx_diagnostics {
class NodeManagerException:public std::runtime_error{
public:
	inline NodeManagerException(std::string message, int node_id):
	std::runtime_error(generateMessage(message, node_id).c_str()){}

private:
	inline std::string generateMessage(std::string& message, int node_id){
		std::stringstream output;
		output<<message<<":"<<node_id;
		return output.str();
	}
};

class NoMatchingNodeId:NodeManagerException{
public:
	NoMatchingNodeId(int node_id):NodeManagerException("node_id does not exist", node_id){};
};

class NonUniqueNodeName:NodeManagerException{
public:
	NonUniqueNodeName(std::string name):NodeManagerException(generateMessage(name), -1){};
private:
	std::string generateMessage(std::string& name){
		std::stringstream output;
		output<<name<<" is not unique";
		return output.str();
	}
};


class NodeManager {
	class NodeInformationContainer;
	typedef NodeManager::NodeInformationContainer container;
	typedef boost::unordered_map<int, container> node_map;
public:
	NodeManager();
	virtual ~NodeManager();

	int registerNode(const std::string& node_name, const std::string& node_type, int criticality, int heartbeat_rate, int heartbeat_tolarence)throw(NonUniqueNodeName);

	const NodeInformationContainer& getNode(int node_id) const throw(NoMatchingNodeId);

	void touch(int node_id)throw(NoMatchingNodeId);

	void deleteNode(int node_id)throw(NoMatchingNodeId);

	void printList(std::stringstream& output) const;

	const container& operator[](int node_id) const throw(NoMatchingNodeId);

	friend std::ostream& operator<<(std::ostream& out, const NodeManager& rhs);


private:
	int next_id_;		///The next node_id to be assigned at registration

	node_map nodes_;	///A Map of node_id's to their respective NodeInformationContainer data

	bool checkId(int node_id)const throw(NoMatchingNodeId);

	/**
	 * Sub-class to act as a container type for holding information about registered nodes
	 */
	class NodeInformationContainer{
	public:
		/**
		 * Empty constructor for STL compliance
		 */
		inline NodeInformationContainer(){
			this->criticality_ = 0;
			this->heartbeat_rate_ = 0;
			this->heartbeat_tolerance_ = 0;
		};

		/**
		 * Copy constructor for STL compliance
		 * @param copy NodeInformationContainer to copy
		 */
		inline NodeInformationContainer(NodeInformationContainer& copy):
					node_name_(copy.node_name_),
					node_type_(copy.node_type_),
					last_touch_(copy.last_touch_){
			this->criticality_ = copy.criticality_;
			this->heartbeat_rate_ = copy.heartbeat_rate_;
			this->heartbeat_tolerance_ = copy.heartbeat_tolerance_;
		}
		inline NodeInformationContainer(const NodeInformationContainer& copy):
							node_name_(copy.node_name_),
							node_type_(copy.node_type_),
							last_touch_(copy.last_touch_){
			this->criticality_ = copy.criticality_;
			this->heartbeat_rate_ = copy.heartbeat_rate_;
			this->heartbeat_tolerance_ = copy.heartbeat_tolerance_;
		}
		/**
		 * Creates a new NodeInformationContainer
		 * @param [in] node_name			The unique ROS name for a node
		 * @param [in] node_type			The non-unique type of the node
		 * @param [in] criticality			The criticality for the node
		 * @param [in] heartbeat_rate		The heartbeat rate for the node, in Hz
		 * @param [in] heartbeat_tolerance	The tolerance amount for the node, in Hz
		 */
		inline NodeInformationContainer(const std::string& node_name, const std::string& node_type, int criticality, int heartbeat_rate, int heartbeat_tolarence):
				node_name_(node_name),
				node_type_(node_type){
			this->criticality_    = criticality;
			this->heartbeat_rate_ = heartbeat_rate;
			this->heartbeat_tolerance_ = heartbeat_tolarence;
			this->last_touch_     = ros::Time::now();
		}
		inline NodeInformationContainer(std::string& node_name, std::string& node_type, int criticality, int heartbeat_rate, int heartbeat_tolarence):
						node_name_(node_name),
						node_type_(node_type){
			this->criticality_    = criticality;
			this->heartbeat_rate_ = heartbeat_rate;
			this->heartbeat_tolerance_ = heartbeat_tolarence;
			this->last_touch_     = ros::Time::now();
		}
		inline NodeInformationContainer(const char* node_name, const char* node_type, int criticality, int heartbeat_rate, int heartbeat_tolarence):
								node_name_(node_name),
								node_type_(node_type){
			this->criticality_    = criticality;
			this->heartbeat_rate_ = heartbeat_rate;
			this->heartbeat_tolerance_ = heartbeat_tolarence;
			this->last_touch_     = ros::Time::now();
		}

		virtual ~NodeInformationContainer(){};

		/**
		 * @return The unique ROS name of the node
		 */
		inline const std::string& getName() const{return this->node_name_;}
		/**
		 * @return The non-unique node-type of the node
		 */
		inline const std::string& getType() const{return this->node_type_;}
		/**
		 * @return The criticality level for the node
		 */
		inline int getCriticality() 		const{return this->criticality_;}
		/**
		 * @return The heartbeat rate for the node, in Hz
		 */
		inline int getHeartbeatRate() 		const{return this->heartbeat_rate_;}
		/**
		 * @return The heartbeat tolerence for the node, in Hz
		 */
		inline int getHeartbeatTolerence()	const{return this->heartbeat_tolerance_;}

		inline const ros::Time& getLastTouch() const{return this->last_touch_;}

		inline void touch(){this->last_touch_ = ros::Time::now();}

		inline const std::string toString() const{
			std::stringstream output;
			output<<this->node_name_<<"["<<this->node_type_<<"]<"<<this->last_touch_<<"> C:"<<this->criticality_<<" H:"<<this->heartbeat_rate_<<" T:"<<this->heartbeat_tolerance_;
			return output.str();
		}

		inline friend std::ostream& operator<<(std::ostream& out, const NodeInformationContainer rhs) // output
		{
			out << rhs.toString();
			return out;
		}

		//		inline friend std::istream& operator>>(std::istream& in, NodeInformationContainer& rhs) // output
		//		{
		//			std::string _t_node_name, _t_node_type;
		//			int _c, _h, _t;
		//			ros::Time _lt;
		//
		//			in>>_t_node_name;
		//			rhs._node_name = _t_node_name;
		//
		//			in.ignore(1);
		//			in>>_t_node_type;
		//			rhs._node_type = _t_node_type;
		//
		//			in.ignore(2);
		//			in>>_lt;
		//			rhs._last_touch = _lt;
		//
		//			in.ignore(4);
		//			in>>_c;
		//			rhs._criticality = _c;
		//
		//			in.ignore(3);
		//			in>>_h;
		//			rhs._heartbeat_rate = _h;
		//
		//			in.ignore(3);
		//			in>>_t;
		//			rhs._heartbeat_tolerance = _t;
		//
		//			return in;
		//		}

	private:
		std::string node_name_;				///Unique ROS name of the node
		std::string node_type_;				///Non-unique node-type of the node
		int			criticality_;			///
		int			heartbeat_rate_;
		int			heartbeat_tolerance_;
		ros::Time   last_touch_;
	};

};

} /* namespace oryx_diagnostics */
#endif /* NODEMANAGER_H_ */
