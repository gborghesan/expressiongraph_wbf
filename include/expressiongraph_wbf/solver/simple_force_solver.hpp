

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

	/** constructor for systems with only scalar joints.
	 *
	 * This is the most normal case, the time index is used
	 * if some expression has time derivative (_e.g._ trajectories)
	 * @param joint_indexes indexes of the joints (all scalar).
	 *  This maps  the input index used in defining expressions,
	 *   to the position of the in the input (position) and output (torque) used in
	 *   #Compute(const std::vector<double> &q_in, double time,Eigen::VectorXd &tau_out)
	 *    or
	 * #Compute(const std::vector<double> &q_in, Eigen::VectorXd &tau_out);
	 * @param time_index index of time, if set to -1, no expression is derived or update w.r.t. time.
	 */
	simple_force_solver(
			const std::vector<int>& joint_indexes,
			const int time_index=-1 );
	/** constructor for systems with  scalar joints and controllable rotations _e.g._free floating bases.
	 *
	 * In this case, we have to give 3 vectors of indexes that are __not__ independent.
	 * @param joint_indexes_for_output indexes used in computing the jacobian and the output,
	 * it mixes torques compute for scalar inputs
	 *  and wrenches computes for rotational inputs.
	 * @param joint_indexes_input_scalar indexes that are used for declaring the scalar inputs,
	 * @param joint_indexes_input_rotation indexes that are used for declaring the rotational inputs.
	 * @param time_index index of time, if set to -1, no expression is derived or update w.r.t. time.
	 *
	 *  __Rotations occupies 3 slots!__
	 *   ###Example: controllable pose:
	 *   If the controllable frame is
	 *   @code
	 *   KDL::Expression<KDL::Frame>::Ptr w_T_ee  = frame( inputRot(4),
	 *	 			KDL::vector(input(1),input(2),input(3)));
	 *	 @endcode
	*	Then, we will use the following indices:
		@code
			joint_indexes_out[0]=1; //force x
			joint_indexes_out[1]=2; //force y
			joint_indexes_out[2]=3; //force z
			joint_indexes_out[3]=4; //wrench x (same index of rotation)
			joint_indexes_out[4]=5; //wrench y (same index of rotation+1)
			joint_indexes_out[5]=6; //wrench z (same index of rotation+2)
			joint_indexes_scalar[0]=1; //position x
			joint_indexes_scalar[1]=2; //position y
			joint_indexes_scalar[2]=3; //position z
			joint_indexes_rot[0]=4; //rotation

			simple_force_solver(joint_indexes_out,
								joint_indexes_scalar,
								joint_indexes_rot));
		 @endcode

	 */
	simple_force_solver(
			const std::vector<int> & joint_indexes_for_output,
			const std::vector<int> & joint_indexes_input_scalar,
			const std::vector<int> & joint_indexes_input_rotation,
			const int time_index=-1 );

	simple_force_solver();

	void setJointIndex(const std::vector<int>&indx);
	void setJointIndex(const std::vector<int>&indx_out,
			const std::vector<int>&indx_scalar,
			const std::vector<int>&indx_rot);
	void setTimeIndex(const int);

	/**@name  Constraint  Handling
	 *@{
	 * */
	/** Add constraints to the solver.
		 * \param name name of the constraint.
		 * \param c constraint to be added
		 * \return false if the name is already used. */
	bool addConstraint(const std::string& name,
			const constraint &c);
	/** Add constraints to the solver.
		 * \param name name of the constraint.
		 * \param c  pointer to constraint to be added
		 * \return false if the name is already used. */
	bool addConstraint(const std::string& name,
			const constraint::Ptr &c);
	/** Remove a constraint from the solver
		 * \param name name of the constraint to be removed
		 * \return false if no cosntraint with such name exists. */
	bool RemoveConstraint(const std::string &s);

	///@}

	int Prepare();

/**@name Public interfaces for computing the force
 *
 * All these functions calls #Compute(const std::vector<double> &q_in, 	const std::vector<KDL::Rotation> &R_in,double time, Eigen::VectorXd &tau_out,bool time_present) .
 * the indexes used for the ordering
 * of the vectors (both input and output) must be the same given to the constructor.
 * @{
 */

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

	typedef boost::shared_ptr<simple_force_solver> Ptr;//<shared pointer.
};
}
