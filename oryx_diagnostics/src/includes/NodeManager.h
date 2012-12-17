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
	int _next_id;		///The next node_id to be assigned at registration

	node_map _nodes;	///A Map of node_id's to their respective NodeInformationContainer data

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
			this->_criticality = 0;
			this->_heartbeat_rate = 0;
			this->_heartbeat_tolerance = 0;
		};

		/**
		 * Copy constructor for STL compliance
		 * @param copy NodeInformationContainer to copy
		 */
		inline NodeInformationContainer(NodeInformationContainer& copy):
					_node_name(copy._node_name),
					_node_type(copy._node_type),
					_last_touch(copy._last_touch){
			this->_criticality = copy._criticality;
			this->_heartbeat_rate = copy._heartbeat_rate;
			this->_heartbeat_tolerance = copy._heartbeat_tolerance;
		}
		inline NodeInformationContainer(const NodeInformationContainer& copy):
							_node_name(copy._node_name),
							_node_type(copy._node_type),
							_last_touch(copy._last_touch){
			this->_criticality = copy._criticality;
			this->_heartbeat_rate = copy._heartbeat_rate;
			this->_heartbeat_tolerance = copy._heartbeat_tolerance;
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
				_node_name(node_name),
				_node_type(node_type){
			this->_criticality    = criticality;
			this->_heartbeat_rate = heartbeat_rate;
			this->_heartbeat_tolerance = heartbeat_tolarence;
			this->_last_touch     = ros::Time::now();
		}
		inline NodeInformationContainer(std::string& node_name, std::string& node_type, int criticality, int heartbeat_rate, int heartbeat_tolarence):
						_node_name(node_name),
						_node_type(node_type){
			this->_criticality    = criticality;
			this->_heartbeat_rate = heartbeat_rate;
			this->_heartbeat_tolerance = heartbeat_tolarence;
			this->_last_touch     = ros::Time::now();
		}
		inline NodeInformationContainer(const char* node_name, const char* node_type, int criticality, int heartbeat_rate, int heartbeat_tolarence):
								_node_name(node_name),
								_node_type(node_type){
			this->_criticality    = criticality;
			this->_heartbeat_rate = heartbeat_rate;
			this->_heartbeat_tolerance = heartbeat_tolarence;
			this->_last_touch     = ros::Time::now();
		}

		virtual ~NodeInformationContainer(){};

		/**
		 * @return The unique ROS name of the node
		 */
		inline const std::string& getName() const{return this->_node_name;}
		/**
		 * @return The non-unique node-type of the node
		 */
		inline const std::string& getType() const{return this->_node_type;}
		/**
		 * @return The criticality level for the node
		 */
		inline int getCriticality() 		const{return this->_criticality;}
		/**
		 * @return The heartbeat rate for the node, in Hz
		 */
		inline int getHeartbeatRate() 		const{return this->_heartbeat_rate;}
		/**
		 * @return The heartbeat tolerence for the node, in Hz
		 */
		inline int getHeartbeatTolerence()	const{return this->_heartbeat_tolerance;}

		inline const ros::Time& getLastTouch() const{return this->_last_touch;}

		inline void touch(){this->_last_touch = ros::Time::now();}

		inline const std::string toString() const{
			std::stringstream output;
			output<<this->_node_name<<"["<<this->_node_type<<"]<"<<this->_last_touch<<"> C:"<<this->_criticality<<" H:"<<this->_heartbeat_rate<<" T:"<<this->_heartbeat_tolerance;
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
		std::string _node_name;				///Unique ROS name of the node
		std::string _node_type;				///Non-unique node-type of the node
		int			_criticality;			///
		int			_heartbeat_rate;
		int			_heartbeat_tolerance;
		ros::Time   _last_touch;
	};

};

} /* namespace oryx_diagnostics */
#endif /* NODEMANAGER_H_ */
