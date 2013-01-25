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

/**
 * Base exception class for things going wrong with a NodeManager
 */
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

/**
 * Class for representing the exception case of when an attempt is made to access an invalid node_id in a NodeManager
 */
class NoMatchingNodeId:public NodeManagerException{
public:
	/**
	 * Constructor for creating a new exception
	 * @param node_id The node_id that caused the exception to be generated
	 */
	NoMatchingNodeId(int node_id):NodeManagerException("node_id does not exist", node_id){};
};

/**
 * Class for representing the exception case for when a non-unique node-name is inserted into the NodeManager
 */
class NonUniqueNodeName:public NodeManagerException{
public:
	NonUniqueNodeName(std::string name):NodeManagerException(generateMessage(name), -1){};
private:
	std::string generateMessage(std::string& name){
		std::stringstream output;
		output<<name<<" is not unique";
		return output.str();
	}
};

/**
 * Class for maintain a set of registered nodes and their associated node information. Provides basic CRUD functionality
 */
class NodeManager {
public:
	class NodeInformationContainer;
	typedef NodeManager::NodeInformationContainer container;	///typedef for convenience
	typedef boost::unordered_map<int, container> node_map;		///typedef for convenience
	typedef std::pair<bool,int> check_result;					///typedef for convenience
	typedef boost::unordered_map<int, check_result> result_map;	///typedef for convenience

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
			this->heartbeat_rate_		= (1.0/(double)heartbeat_rate);
			this->heartbeat_tolerance_	= (1.0/(double)heartbeat_tolarence);
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
		 * @return The heartbeat rate for the node, in seconds
		 */
		inline double getHeartbeatRate() 		const{return this->heartbeat_rate_;}
		/**
		 * @return The heartbeat tolerance for the node, in seconds
		 */
		inline double getHeartbeatTolerence()	const{return this->heartbeat_tolerance_;}

		/**
		 * @return The last ros::Time that the node was touched
		 */
		inline const ros::Time& getLastTouch() const{return this->last_touch_;}

		/**
		 * Touch the node
		 */
		inline void touch(){this->last_touch_ = ros::Time::now();}

		/**
		 * Converts the NodeContainer into a string representation of the format "node_name[node_type]<last_touch> C: criticality H:heartbeat_rate T: heartbeat_tolerence"
		 * @return A std::string containing the string representation of the NodeContainer
		 */
		inline const std::string toString() const{
			std::stringstream output;
			output<<this->node_name_<<"["<<this->node_type_<<"]<"<<this->last_touch_<<"> C:"<<this->criticality_<<" H:"<<(1.0/this->heartbeat_rate_)<<" T:"<<(1.0/this->heartbeat_tolerance_);
			return output.str();
		}

		/**
		 * Overload of the << operator for use with ostreams. Same output format as NodeContainer::toString()
		 * @param [in/out]	out The std::ostream to insert into
		 * @param [in] 		rhs The NodeInformationContainer to insert into an ostream
		 * @return The inputed ostream with the NodeInformationContainer's string representation inserted into it
		 */
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
		int			criticality_;			///The criticality level of the node
		double		heartbeat_rate_;		///The heartbeat rate of the node, in seconds
		double		heartbeat_tolerance_;	///The heartbeat tolerence of the node, in seconds
		ros::Time   last_touch_;			///The ros::Time corresponding to the last time the node was touched
	};

	/**
	 * Sets up a new NodeManager with an empty set of nodes.
	 */
	NodeManager();
	virtual ~NodeManager();

	/**
	 * Registers a new node with the NodeManager
	 * @param [in] node_name			The unique ROS name for a node
	 * @param [in] node_type			The non-unique type of the node
	 * @param [in] criticality			The criticality for the node
	 * @param [in] heartbeat_rate		The heartbeat rate for the node, in Hz
	 * @param [in] heartbeat_tolerance	The tolerance amount for the node, in Hz
	 * @return The unique node_id for the newly registered node
	 */
	int registerNode(const std::string& node_name, const std::string& node_type, int criticality, int heartbeat_rate, int heartbeat_tolarence)throw(NonUniqueNodeName);

	/**
	 * Retrieves the information on a node based on a given node_id
	 * @param [in] node_id The node_id to search for
	 * @return The NodeInformationContainer which holds the information on that node
	 * @throw NoMatchingNodeId if the given node_id was not registered with the system
	 */
	const NodeInformationContainer& getNode(int node_id) const throw(NoMatchingNodeId);

	/**
	 * Touches a node in the NodeManager
	 * @param [in] node_id The node_id to touch
	 * @throw NoMatchingNodeId if the given node_id was not registered with the system
	 */
	void touch(int node_id)throw(NoMatchingNodeId);

	/**
	 * De-registers a node with the NodeManager
	 * @param [in] node_id The node_id to touch
	 * @throw NoMatchingNodeId if the given node_id was not registered with the system
	 */
	void deleteNode(int node_id)throw(NoMatchingNodeId);

	/**
	 * Prints out a list of the information on all currently registered nodes in the NodeManager
	 * @param [out] output std::stringstream to write the list to
	 */
	void printList(std::stringstream& output) const;


	/**
	 * Checks to see if the nodes in the NodeManager have been touched recently enough to pass their heartbeat
	 * rate within their specified tolerence
	 * @param [out] results A reference to an empty result_map to write the results into
	 * @param [in]  checkTime The time to compare against
	 *
	 * @details The results map will be filled with check_results as values, and keyed by node_id. The first item in the check_result is
	 * a bool, which will be true if it passed its heartbeat rate test. The second item is an int, an corrisponds to the criticality of that node
	 */
	void timeCheck(result_map& results, ros::Time& checkTime) const;


	/**
	 * Retrieves the information on a node based on a given node_id
	 * @param [in] node_id The node_id to search for
	 * @return The NodeInformationContainer which holds the information on that node
	 * @throw NoMatchingNodeId if the given node_id was not registered with the system
	 */
	const container& operator[](int node_id) const throw(NoMatchingNodeId);

	/**
	 * Overload of << operator for use with std::ostream. Has the same operation as printList
	 * @param [in/out] out std::stringstream to write the list to
	 * @param [in] rhs the NodeManager to insert into the stream
	 */
	friend std::ostream& operator<<(std::ostream& out, const NodeManager& rhs);


private:
	int next_id_;		///The next node_id to be assigned at registration

	node_map nodes_;	///A Map of node_id's to their respective NodeInformationContainer data

	/**
	 * Checks to make sure that a given node_id exists in the NodeManager's node_map
	 * @param node_id The node_id to use as a key to check for
	 * @return TRUE if the key is in the map, else false
	 * @throw NoMatchingNodeId if the node_id is not in the map
	 */
	bool checkId(int node_id)const throw(NoMatchingNodeId);
};

} /* namespace oryx_diagnostics */
#endif /* NODEMANAGER_H_ */
