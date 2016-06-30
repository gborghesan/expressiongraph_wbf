#ifndef _CONSTRAINTS_HPP_
#define _CONSTRAINTS_HPP_

#include <kdl/expressiontree.hpp>
#include <expressiongraph_wbf/solver/controller_base.hpp>
#include <expressiongraph_wbf/solver/space_description.hpp>
#include <exception>

using namespace std;

struct constraintException : public exception{};
struct orderException : public constraintException{
	const char * what () const throw (){
		return "Order Exception";
	}};
struct someIndexEqualToTimeIndexException : public constraintException{
	const char * what () const throw (){
		return "some Index in joint indexes vector is equal to time index";
	}};
struct someIndexIsNegativeException : public constraintException{
	const char * what () const throw (){
		return "One or more Joint Index is negative";
	}};
struct JointIndexSizeException : public constraintException{
	const char * what () const throw (){
		return "Joint Index Sizes does not agree";
	}};
struct negativePriorityException : public constraintException{
	const char * what () const throw (){
		return "At least one priority is negative";
	}};
struct jointWeightException : public constraintException{
	const char * what () const throw (){
		return "joint weight vector should be the same size of jonit number, and striclty positive";
	}};

namespace wbf{
typedef struct constraint
{
	space_description::Ptr space;
	controller::Ptr ctrl;
	controller::Ptr ctrl_lower;
	unsigned int priority_level;
	Expression<double>::Ptr weight;


	typedef boost::shared_ptr<constraint> Ptr;
	constraint(
			space_description::Ptr _space,
			controller::Ptr _ctrl,
			unsigned int _priority_level=2,
			Expression<double>::Ptr _weight=Constant(1.0)
											){
		space=_space;
		ctrl=_ctrl;
		priority_level=_priority_level;
		weight=_weight;
		ctrl_lower=_ctrl;
	}
	constraint(
			space_description::Ptr _space,
			controller::Ptr _ctrl,
			Expression<double>::Ptr _weight
			){
		space=_space;
		ctrl=_ctrl;
		priority_level=2;
		weight=_weight;
		ctrl_lower=_ctrl;
	}
	constraint(
			space_description::Ptr _space,
			controller::Ptr _ctrl_lower,
			controller::Ptr _ctrl_upper,
			unsigned int _priority_level=0,
			Expression<double>::Ptr _weight=Constant(1.0)
											){
		space=_space;
		ctrl=_ctrl_upper;
		priority_level=_priority_level;
		weight=_weight;
		ctrl_lower=_ctrl_lower;
	}
	constraint(const constraint&c){
		space=c.space;
		ctrl=c.ctrl;
		priority_level=c.priority_level;
		weight=c.weight;
		ctrl_lower=c.ctrl_lower;
	}

}constraint;



typedef std::map <std::string, constraint::Ptr> c_map_type;
inline bool check_constraint_validity(const constraint & cs)
{
	if (cs.ctrl->output_size()!=cs.space->output_size()) return false;
	if (cs.ctrl_lower)
		if(cs.ctrl->output_size()!=cs.ctrl_lower->output_size())
			return false;
	return true;
}
inline bool check_constraint_validity(const constraint::Ptr & cs)
{
	return check_constraint_validity(*cs);
}


class constraints{
private:

	std::vector<int> joint_indexes_for_output;
	std::vector<int> joint_indexes_input_scalar;
	std::vector<int> joint_indexes_input_rotation;
	int timeIndex;
	unsigned int n_of_joints;

	/// constrainMapPerPriority is a vector of maps,
	/// where priorities are divided by priority
	std::vector< c_map_type> constrainMapPerPriority;
	std::vector<unsigned int> constraintsPerPriority;
	std::vector<Eigen::MatrixXd> JacobianPerPriority;
	std::vector<Eigen::VectorXd> lowerBoundPerPriority;
	std::vector<Eigen::VectorXd> upperBoundPerPriority;
	std::vector<Eigen::VectorXd> WyPerPriority;
	Eigen::MatrixXd J1,J3, J6;
	Eigen::VectorXd y1l,y3l,y6l;
	Eigen::VectorXd y1u,y3u,y6u;
	bool prepared;
	Eigen::VectorXd Wqdiag;

	void checkJointIndex();

public:
	constraints(){timeIndex=-1;prepared=false;}
	constraints(
			const std::vector<int>& _joint_indexes_for_output,
			const std::vector<int>& _joint_indexes_input_scalar,
			const std::vector<int>& _joint_indexes_input_rotation,
			const int timeIndex=-1);

	constraints(
			const std::vector<int>& joint_indexes,
			const int timeindex=-1);


	/**@name  Constraint  Handling
	 *@{
	 * */
	/** Add constraint to the constrains.
		 * \param name name of the constraint.
		 * \param c constraint to be added
		 * \return false if the name is already used. */
	bool addConstraint(const std::string& name,
					   const constraint &c);
	/** Add constraints to the constrains.
		 * \param name name of the constraint.
		 * \param c  pointer to constraint to be added
		 * \return false if the name is already used. */
	bool addConstraint(const std::string& name,
					   const constraint::Ptr &c);
	/** Remove a constraint from the constrains
		 * \param name name of the constraint to be removed
		 * \return false if no cosntraint with such name exists. */
	bool RemoveConstraint(const std::string &name);

	///@}


	/** @name  Setter functions
	 * All this function set the object in a un uprepared state
	 * so the Prepare() must be called again
	 *@{
	 * */
	/**
	 * @brief setJointIndex
	 * set the joint indexes, assuming that no input is a rotation
	 * see
	 * @param joint_indexes
	 */
	void setJointIndex(const std::vector<int>&joint_indexes);
	/**
	 * @brief setJointIndex
	 * Set the joint index, in case rotation is used.
	 * @param joint_indexes_for_output
	 * @param joint_indexes_input_scalar
	 * @param joint_indexes_input_rotation
	 */
	void setJointIndex(
			const std::vector<int>& joint_indexes_for_output,
			const std::vector<int>& joint_indexes_input_scalar,
			const std::vector<int>& joint_indexes_input_rotation);
	/**
	 * @brief setTimeIndex
	 * set the index to derive toward time.
	 * This index is used only in the expressions of \ref controller
	 * @param timeIndex
	 */
	void setTimeIndex(const int timeIndex);
	/**
	 * @brief setQweights
	 * Set joint weights. If the function is not used, Ones will be used.
	 * @param Wqdiag
	 */
	void setQweights(const Eigen::VectorXd& Wqdiag);
	///@}
	/** @name  Getter functions
	 * This function returns true if the object is not prepared
	 *@{
	 * */
	bool getQweights( Eigen::VectorXd& _Wqdiag)const{
		if (!prepared) return false;
		_Wqdiag=Wqdiag;
		return true;
	}
	bool getPriorityCardinality(std::vector<unsigned int> &_constraintsPerPriority )const{
		if (!prepared) return false;
		_constraintsPerPriority=constraintsPerPriority;
		return true;
	}
	bool getJacobian(std::vector<Eigen::MatrixXd> &_JacobianPerPriority )const{
		if (!prepared) return false;
		_JacobianPerPriority=JacobianPerPriority;
		return true;
	}
	bool getLowerBounds(std::vector<Eigen::VectorXd> &_lowerBoundPerPriority )const{
		if (!prepared) return false;
		_lowerBoundPerPriority=lowerBoundPerPriority;
		return true;
	}
	bool getUpperBounds(std::vector<Eigen::VectorXd> &_upperBoundPerPriority )const{
		if (!prepared) return false;
		_upperBoundPerPriority=upperBoundPerPriority;
		return true;
	}
	bool getYweights(std::vector<Eigen::VectorXd> &_WyPerPriority )const{
		if (!prepared) return false;
		_WyPerPriority=WyPerPriority;
		return true;
	}
	///@}
	void Prepare();
	int computeJacobianAndBounds(
			const std::vector<double> &q_in,
			const std::vector<Rotation> &R_in,
			double time,
			bool time_present);
	int computeJacobianAndBounds(
			const std::vector<double> &q_in,
			double time,
			bool time_present)
	{
		std::vector<Rotation> R_in;
		return  computeJacobianAndBounds(
					q_in,R_in,time,
					time_present);
	}



	typedef boost::shared_ptr<constraints> Ptr;
};




}//namespace
#endif
