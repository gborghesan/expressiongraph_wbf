

#include "expressiongraph_wbf/solver/constraints.hpp"
#include "expressiongraph_wbf/solver/controller_base.hpp"
//#include <map>

using namespace wbf;
using namespace std;
using namespace KDL;
namespace wbf {
typedef map <std::string, constraint::Ptr> c_map_type;
class simple_force_solver{
private:
	std::vector<int> joint_indexes_for_output;
	std::vector<int> joint_indexes_input_scalar;
	std::vector<int> joint_indexes_input_rotation;
	int time_index;
	unsigned int n_of_joints;
	unsigned int n_of_output;
	c_map_type c_map;
	Eigen::MatrixXd J;
	Eigen::MatrixXd Jt;
	Eigen::MatrixXd J1,J3,J6;
	Eigen::VectorXd lambda_des;
	Eigen::VectorXd lambda1,lambda3,lambda6;
	/** Prepared is set to false each time some constraint is added or removed
	 * */
	bool prepared;
	/** Computes the joint torques.
	 *
	 * Private function that is called after
	 * some configuration from the public interface;
	 * it computes the desired torque to be applied,
	 *  given the desired force computed by constraints.
	 * \param q_in vector of input (scalars).
	 * \param R_in vector of input (Rotation).
	 * \param time time input (used if time_present=true).
	 * \param tau_out computed force to be actuated.
	 * \param time_present is set to true if the time
	 * 			is used (_e.g._ for feed-forward terms of trajectories)
	 * \return 1 if it ok, otherwise error code (not yet documented).
	 * */
	int Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			double time,
			Eigen::VectorXd &tau_out,
			bool time_present);
public:
	//constructor for input rotational
	simple_force_solver(
			const std::vector<int> & _joint_indexes_for_output,
			const std::vector<int> & _joint_indexes_input_scalar,
			const std::vector<int> & _joint_indexes_input_rotation,
			const int time_index=-1 );
	//constructor for only scalar
	simple_force_solver(
			const std::vector<int>& joint_indexes,
			const int time_index=-1 );
	simple_force_solver();

	void setJointIndex(const std::vector<int>&indx);
	void setJointIndex(const std::vector<int>&indx_out,
			const std::vector<int>&indx_scalar,
			const std::vector<int>&indx_rot);
	void setTimeIndex(const int);
	///@{
	/** Add constraints to the solver.
		 * \param name name of the constraint.
		 * \param c constraint or pointer to constraint to be added
		 * \return false if the name is already used. */
	bool addConstraint(const std::string& name,
			const constraint &c);
	bool addConstraint(const std::string& name,
			const constraint::Ptr &c);
	///@}
	bool RemoveConstraint(const std::string &s);
	int Prepare();


	///@{
	/**Public interfaces for computing the force. */
	/** Compute for robot whose joints are __scalar__,
	and some expressions are time dependent.*/
	int Compute(const std::vector<double> &q_in, double time,
			Eigen::VectorXd &tau_out);
	/** Compute for robot whose joints are __scalar__,
	and no expression is time dependent.*/
	int Compute(const std::vector<double> &q_in,Eigen::VectorXd &tau_out);
	/** Compute for robot whose joints are __scalar AND rotational__
	 * (_e.g._ free-floating base)
	and some expressions are time dependent.*/
	int Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			double time,
			Eigen::VectorXd &tau_out);
	/** Compute for robot whose joints are __scalar AND rotational__
	 * (_e.g._ free-floating base)
	and no expression is time dependent.*/
	int Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			Eigen::VectorXd &tau_out);
	///@}
	//memory assignment copy...
	Eigen::VectorXd getLastDesiredForce(){return lambda_des;};

	typedef boost::shared_ptr<simple_force_solver> Ptr;
};
}
